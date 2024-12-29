// Hyperbolic Rogue -- basic computations in non-Euclidean geometry
// Copyright (C) 2011-2019 Zeno Rogue, see 'hyper.cpp' for details

/** \file hyperpoint.cpp
 *  \brief basic computations in non-Euclidean geometry
 *
 *  This implements hyperpoint (a point in non-Euclidean space), transmatrix (a transformation matrix),
 *  and various basic routines related to them: rotations, translations, inverses and determinants, etc.
 *  For nonisotropic geometries, it rather refers to nonisotropic.cpp.
 */

#include "hyper.h"
namespace hr {

#if HDR

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

static constexpr ld A_PI = M_PI;
static constexpr ld TAU = 2 * A_PI;
static constexpr ld degree = A_PI / 180;
static const ld golden_phi = (sqrt(5)+1)/2;
static const ld log_golden_phi = log(golden_phi);

constexpr ld operator""_deg(long double deg) { return deg * A_PI / 180; }
#endif

eGeometry geometry;
eVariation variation;


#if HDR
/** \brief A point in our continuous space
 *
 *  Originally used for representing points in the hyperbolic plane.
 *  Currently used for all kinds of supported spaces, as well as
 *  for all vector spaces (up to 4 dimensions). We are using
 *  the normalized homogeneous coordinates, which allows us to work with most
 *  geometries in HyperRogue in a uniform way.

 *  In the hyperbolic plane, this is the Minkowski hyperboloid model:
 *  (x,y,z) such that x*x+y*y-z*z == -1 and z > 0.
 *
 *  In spherical geometry, we have x*x+y*y+z*z == 1.
 *
 *  In Euclidean geometry, we have z = 1.
 *
 *  In isotropic 3D geometries an extra coordinate is added.
 *
 *  In nonisotropic coordinates h[3] == 1.
 *
 *  In product geometries the 'z' coordinate is modelled by multiplying all 
 *  three coordinates with exp(z).
 *
 */

struct hyperpoint : array<ld, MAXMDIM> {
  hyperpoint() {}
  
  #if MAXMDIM == 4
  constexpr hyperpoint(ld x, ld y, ld z, ld w) : array<ld, MAXMDIM> {{x,y,z,w}} {}
  #else
  constexpr hyperpoint(ld x, ld y, ld z, ld w) : array<ld, MAXMDIM> {{x,y,z}} {}
  #endif

  inline hyperpoint& operator *= (ld d) {
    for(int i=0; i<MXDIM; i++) self[i] *= d;
    return self;
    }
  
  inline hyperpoint& operator /= (ld d) { 
    for(int i=0; i<MXDIM; i++) self[i] /= d;
    return self;
    }
  
  inline hyperpoint& operator += (const hyperpoint h2) { 
    for(int i=0; i<MXDIM; i++) self[i] += h2[i];
    return self;
    }

  inline hyperpoint& operator -= (const hyperpoint h2) { 
    for(int i=0; i<MXDIM; i++) self[i] -= h2[i];
    return self;
    }

  inline friend hyperpoint operator * (ld d, hyperpoint h) { return h *= d; }  
  inline friend hyperpoint operator * (hyperpoint h, ld d) { return h *= d; }  
  inline friend hyperpoint operator / (hyperpoint h, ld d) { return h /= d; }  
  inline friend hyperpoint operator + (hyperpoint h, hyperpoint h2) { return h += h2; }
  inline friend hyperpoint operator - (hyperpoint h, hyperpoint h2) { return h -= h2; }

  inline friend hyperpoint operator - (hyperpoint h) { return h * -1; }

  // cross product  
  inline friend hyperpoint operator ^ (hyperpoint h1, hyperpoint h2) {
    return hyperpoint(
      h1[1] * h2[2] - h1[2] * h2[1],
      h1[2] * h2[0] - h1[0] * h2[2],
      h1[0] * h2[1] - h1[1] * h2[0],
      0
      );
    }

  friend ld dot_d(int c, hyperpoint h1, hyperpoint h2) {
    ld sum = 0;
    for(int i=0; i<c; i++) sum += h1[i] * h2[i];
    return sum;
    }

  // Euclidean inner product
  inline friend ld operator | (hyperpoint h1, hyperpoint h2) {
    return dot_d(MXDIM, h1, h2);
    }    
  };

/** \brief A matrix acting on hr::hyperpoint 
 *
 *  Since we are using homogeneous coordinates for hr::hyperpoint,
 *  rotations and translations can be represented
 *  as matrix multiplications. Other applications of matrices in HyperRogue 
 *  (in dimension up to 4) are also implemented using transmatrix.
 */
struct transmatrix {
  ld tab[MAXMDIM][MAXMDIM];
  hyperpoint& operator [] (int i) { return (hyperpoint&)tab[i][0]; }
  const hyperpoint& operator [] (int i) const { return (const hyperpoint&)tab[i]; }
  
  inline friend hyperpoint operator * (const transmatrix& T, const hyperpoint& H) {
    hyperpoint z;
    for(int i=0; i<MXDIM; i++) {
      z[i] = 0;
      for(int j=0; j<MXDIM; j++) z[i] += T[i][j] * H[j];
      }
    return z;
    }

  inline friend transmatrix operator * (const transmatrix& T, const transmatrix& U) {
    transmatrix R;
    for(int i=0; i<MXDIM; i++) for(int j=0; j<MXDIM; j++) {
      R[i][j] = 0;
      for(int k=0; k<MXDIM; k++)
        R[i][j] += T[i][k] * U[k][j];
      }
    return R;
    }  
  };

/** @brief hyperpoint with shift 
 *  shift has two uses:
 *  (1) in the 'universal cover of SL' geometry, shift is used for the extra angular coordinate
 *  (2) in band models, shift is used to draw faraway points correctly
 */
struct shiftpoint {
  hyperpoint h;
  ld shift;
  shiftpoint() {}
  shiftpoint(hyperpoint _h, ld _shift) : h(_h), shift(_shift) {}
  ld& operator [] (int i) { return h[i]; }
  const ld& operator [] (int i) const { return h[i]; }
  inline friend shiftpoint operator + (const shiftpoint& h, const hyperpoint& h2) {
    return shiftpoint{h.h+h2, h.shift};
    }
  inline friend shiftpoint operator - (const shiftpoint& h, const hyperpoint& h2) {
    return shiftpoint{h.h-h2, h.shift};
    }
  };

inline shiftpoint shiftless(const hyperpoint& h, ld shift = 0) {
  shiftpoint res; res.h = h; res.shift = shift; return res;
  }

struct shiftmatrix {
  transmatrix T;
  ld shift;
  shiftmatrix() {}
  shiftmatrix(const transmatrix& _h, ld _shift) : T(_h), shift(_shift) {}
  hyperpoint& operator [] (int i) { return T[i]; }
  const hyperpoint& operator [] (int i) const { return T[i]; }
  inline friend shiftpoint operator * (const shiftmatrix& T, const hyperpoint& h) {
    return shiftpoint{T.T*h, T.shift};
    }
  inline friend shiftmatrix operator * (const shiftmatrix& T, const transmatrix& U) {
    return shiftmatrix{T.T*U, T.shift};
    }
  };

struct shiftmatrix_or_null : shiftmatrix {
  bool is_null;
  shiftmatrix_or_null& operator = (const shiftmatrix& T) { ((shiftmatrix&) self) = T; is_null = false; return self; }
  shiftmatrix_or_null() { is_null = true; }
  };

inline shiftmatrix shiftless(const transmatrix& T, ld shift = 0) {
  shiftmatrix res; res.T = T; res.shift = shift; return res;
  }

/** returns a diagonal matrix */
constexpr transmatrix diag(ld a, ld b, ld c, ld d) {
  #if MAXMDIM==3
  return transmatrix{{{a,0,0}, {0,b,0}, {0,0,c}}};
  #else
  return transmatrix{{{a,0,0,0}, {0,b,0,0}, {0,0,c,0}, {0,0,0,d}}};
  #endif
  }

constexpr hyperpoint Hypc = hyperpoint(0, 0, 0, 0);

/** identity matrix */
constexpr transmatrix Id = diag(1,1,1,1);

/** zero matrix */
constexpr transmatrix Zero = diag(0,0,0,0);

/** a transmatrix with 2D and 3D version, useful for configuration */
struct trans23 {
  transmatrix v2, v3;
  transmatrix& get() { return MDIM == 3 ? v2 : v3; }
  const transmatrix& get() const { return MDIM == 3 ? v2 : v3; }
  trans23() { v2 = Id; v3 = Id; }
  trans23(const transmatrix& T) { v2 = T; v3 = T; }
  trans23(const transmatrix& T2, const transmatrix& T3) { v2 = T2; v3 = T3; }
  bool operator == (const trans23& b) const;
  bool operator != (const trans23& b) const { return !(self == b); }
  trans23 operator * (trans23 T) {
    trans23 t;
    auto& dim = cginf.g.homogeneous_dimension;
    dynamicval<int> d1(dim, dim);
    dim = 3; t.v2 = v2 * T.v2;
    dim = 4; t.v3 = v3 * T.v3;
    return t;
    }
  friend trans23 operator * (transmatrix M, trans23 T) {
    trans23 t(M);
    return t * T;
    }
  };

/** mirror image */
constexpr transmatrix Mirror = diag(1,-1,1,1);

/** mirror image: flip in the Y coordinate */
constexpr transmatrix MirrorY = diag(1,-1,1,1);

/** mirror image: flip in the X coordinate */
constexpr transmatrix MirrorX = diag(-1,1,1,1);

/** mirror image: flip in the Z coordinate */
constexpr transmatrix MirrorZ = diag(1,1,-1,1);

/** rotate by PI in the XY plane */
constexpr transmatrix pispin = diag(-1,-1,1,1);

/** central symmetry matrix */
constexpr transmatrix centralsym = diag(-1,-1,-1,-1);

inline hyperpoint hpxyz(ld x, ld y, ld z) { return MDIM == 3 ? hyperpoint(x,y,z,0) : hyperpoint(x,y,0,z); }
inline hyperpoint hpxyz3(ld x, ld y, ld z, ld w) { return MDIM == 3 ? hyperpoint(x,y,w,0) : hyperpoint(x,y,z,w); }
constexpr hyperpoint point3(ld x, ld y, ld z) { return hyperpoint(x,y,z,0); }
constexpr hyperpoint point30(ld x, ld y, ld z) { return hyperpoint(x,y,z,0); }
constexpr hyperpoint point31(ld x, ld y, ld z) { return hyperpoint(x,y,z,1); }
constexpr hyperpoint point2(ld x, ld y) { return hyperpoint(x,y,0,0); }

constexpr hyperpoint C02 = hyperpoint(0,0,1,0);
constexpr hyperpoint C03 = hyperpoint(0,0,0,1);

/** C0 is the origin in our space */
#define C0 (MDIM == 3 ? C02 : C03)
#endif

bool trans23::operator == (const trans23& b) const { return eqmatrix(v2, b.v2) && eqmatrix(v3, b.v3); }

// basic functions and types
//===========================

EX ld squar(ld x) { return x*x; }

EX int sig(int z) { return ginf[geometry].g.sig[z]; }

EX int curvature() {
  switch(cgclass) {
    case gcEuclid: return 0;
    case gcHyperbolic: return -1;
    case gcSphere: return 1;
    case gcProduct: return PIU(curvature());
    default: return 0;
    }
  }

EX ld sin_auto(ld x) {
  switch(cgclass) {
    case gcEuclid: return x;
    case gcHyperbolic: return sinh(x);
    case gcSphere: return sin(x);
    case gcProduct: return PIU(sin_auto(x));
    case gcSL2: return sinh(x);
    default: return x;
    }
  }

EX ld asin_auto(ld x) {
  switch(cgclass) {
    case gcEuclid: return x;
    case gcHyperbolic: return asinh(x);
    case gcSphere: return asin(x);
    case gcProduct: return PIU(asin_auto(x));
    case gcSL2: return asinh(x);
    default: return x;
    }
  }

EX ld acos_auto(ld x) {
  switch(cgclass) {
    case gcHyperbolic: return acosh(x);
    case gcSphere: return acos(x);
    case gcProduct: return PIU(acos_auto(x));
    case gcSL2: return acosh(x);
    default: return x;
    }
  }

/** \brief volume of a three-dimensional ball of radius r in the current isotropic geometry */
EX ld volume_auto(ld r) {
  switch(cgclass) {
    case gcEuclid: return r * r * r * 240._deg;
    case gcHyperbolic: return M_PI * (sinh(2*r) - 2 * r);
    case gcSphere: return M_PI * (2 * r - sin(2*r));
    default: return 0;
    }
  }

/** \brief area of a circle of radius r in the current isotropic geometry */
EX ld area_auto(ld r) {
  switch(cgclass) {
    case gcEuclid: return r * r * M_PI;
    case gcHyperbolic: return TAU * (cosh(r) - 1);
    case gcSphere: return TAU * (1 - cos(r));
    default: return 0;
    }
  }

/** \brief volume in 3D, area in 2D */
EX ld wvolarea_auto(ld r) {
  if(WDIM == 3) return volume_auto(r);
  else return area_auto(r);
  }

EX ld asin_clamp(ld x) { return x>1 ? 90._deg : x<-1 ? -90._deg : std::isnan(x) ? 0 : asin(x); }

EX ld acos_clamp(ld x) { return x>1 ? 0 : x<-1 ? M_PI : std::isnan(x) ? 0 : acos(x); }

EX ld asin_auto_clamp(ld x) {
  switch(cgclass) {
    case gcEuclid: return x;
    case gcHyperbolic: return asinh(x);
    case gcSL2: return asinh(x);
    case gcSphere: return asin_clamp(x);
    case gcProduct: return PIU(asin_auto_clamp(x));
    default: return x;
    }
  }

EX ld acos_auto_clamp(ld x) {
  switch(cgclass) {
    case gcHyperbolic: return x < 1 ? 0 : acosh(x);
    case gcSL2: return x < 1 ? 0 : acosh(x);
    case gcSphere: return acos_clamp(x);
    case gcProduct: return PIU(acos_auto_clamp(x));
    default: return x;
    }
  }

EX ld cos_auto(ld x) {
  switch(cgclass) {
    case gcEuclid: return 1;
    case gcHyperbolic: return cosh(x);
    case gcSL2: return cosh(x);
    case gcSphere: return cos(x);
    case gcProduct: return PIU(cos_auto(x));
    default: return 1;
    }
  }

EX ld tan_auto(ld x) {
  switch(cgclass) {
    case gcEuclid: return x;
    case gcHyperbolic: return tanh(x);
    case gcSphere: return tan(x);
    case gcProduct: return PIU(tan_auto(x));
    case gcSL2: return tanh(x);
    default: return 1;
    }
  }

EX ld atan_auto(ld x) {
  switch(cgclass) {
    case gcEuclid: return x;
    case gcHyperbolic: return atanh(x);
    case gcSphere: return atan(x);
    case gcProduct: return PIU(atan_auto(x));
    case gcSL2: return atanh(x);
    default: return x;
    }
  }

EX ld atan2_auto(ld y, ld x) {
  switch(cgclass) {
    case gcEuclid: return y/x;
    case gcHyperbolic: return atanh(y/x);
    case gcSL2: return atanh(y/x);
    case gcSphere: return atan2(y, x);
    case gcProduct: return PIU(atan2_auto(y, x));    
    default: return y/x;
    }
  }

/** This function returns the length of the edge opposite the angle alpha in 
 *  a triangle with angles alpha, beta, gamma. This is called the cosine rule,
 *  and of course works only in non-Euclidean geometry. */ 
EX ld edge_of_triangle_with_angles(ld alpha, ld beta, ld gamma) {
  return acos_auto((cos(alpha) + cos(beta) * cos(gamma)) / (sin(beta) * sin(gamma)));
  }

EX hyperpoint hpxy(ld x, ld y) { 
  if(embedded_plane) {
    geom3::light_flip(true);
    hyperpoint h = hpxy(x, y);
    geom3::light_flip(false);
    return cgi.emb->base_to_actual(h);
    }
  if(sl2) return hyperpoint(x, y, 0, sqrt(1+x*x+y*y));
  if(mtwisted) return hyperpoint(x, y, 0, sqrt(1-x*x-y*y));
  return PIU(hpxyz(x,y, translatable ? 1 : sphere ? sqrt(1-x*x-y*y) : sqrt(1+x*x+y*y)));
  }

EX hyperpoint hpxy3(ld x, ld y, ld z) { 
  return hpxyz3(x,y,z, sl2 ? sqrt(1+x*x+y*y-z*z) :translatable ? 1 : sphere ? sqrt(1-x*x-y*y-z*z) : sqrt(1+x*x+y*y+z*z));
  }

#if HDR
// a point (I hope this number needs no comments ;) )
constexpr hyperpoint Cx12 = hyperpoint(1,0,1.41421356237,0);
constexpr hyperpoint Cx13 = hyperpoint(1,0,0,1.41421356237);

#define Cx1 (GDIM==2?Cx12:Cx13)
#endif

EX bool zero_d(int d, hyperpoint h) { 
  for(int i=0; i<d; i++) if(h[i]) return false;
  return true;
  }

/** inner product in the current geometry */

EX ld geo_inner(const hyperpoint &h1, const hyperpoint &h2) {
  ld res = 0;
  for(int i=0; i<MDIM; i++) res += h1[i] * h2[i] * sig(i);
  return res;
  }

/** this function returns approximate square of distance between two points
 *  (in the spherical analogy, this would be the distance in the 3D space,
 *  through the interior, not on the surface)
 *  also used to verify whether a point h1 is on the hyperbolic plane by using Hypc for h2
 */

EX ld intval(const hyperpoint &h1, const hyperpoint &h2) {
  ld res = 0;
  for(int i=0; i<MDIM; i++) res += squar(h1[i] - h2[i]) * sig(i);
  if(elliptic) {
    ld res2 = 0;
    for(int i=0; i<MDIM; i++) res2 += squar(h1[i] + h2[i]) * sig(i);
    return min(res, res2);
    }
  return res;
  }

EX ld quickdist(const hyperpoint &h1, const hyperpoint &h2) {
  if(gproduct) return hdist(h1, h2);
  return intval(h1, h2);
  }

/** square Euclidean hypotenuse in the first d dimensions */
EX ld sqhypot_d(int d, const hyperpoint& h) {
  ld sum = 0;
  for(int i=0; i<d; i++) sum += h[i]*h[i];
  return sum;
  }
  
/** Euclidean hypotenuse in the first d dimensions */
EX ld hypot_d(int d, const hyperpoint& h) {
  return sqrt(sqhypot_d(d, h));
  }

/** @brief h1 and h2 define a line; to_other_side(h1,h2)*x is x moved orthogonally to this line, by double the distance from C0 
 *  (I suppose it could be done better)
 */
EX transmatrix to_other_side(hyperpoint h1, hyperpoint h2) {

  if(cgi.emb->is_sph_in_low() && !geom3::flipped) {
    geom3::light_flip(true);
    h1 = normalize(h1);
    h2 = normalize(h2);
    transmatrix T = to_other_side(h1, h2);
    fix4(T);
    geom3::light_flip(false);
    return T;
    }

  if(sol && meuclid) {
    /* works in 4x4... */
    return gpushxto0(h1) * gpushxto0(h2);
    }
  
  ld d = hdist(h1, h2);

  hyperpoint v;  
  if(euclid)
    v = (h2 - h1) / d;    
  else
    v = (h1 * cos_auto(d) - h2) / sin_auto(d);

  ld d1;
  if(euclid)
    d1 = -(v|h1) / (v|v);
  else
    d1 = atan_auto(-v[LDIM] / h1[LDIM]);
  
  hyperpoint hm = h1 * cos_auto(d1) + (sphere ? -1 : 1) * v * sin_auto(d1);
  
  return rspintox(hm) * xpush(-hdist0(hm) * 2) * spintox(hm);
  }

/** @brief positive for a material vertex, 0 for ideal vertex, negative for ultra-ideal vertex */
EX ld material(const hyperpoint& h) {
  if(sphere || in_s2xe()) return intval(h, Hypc);
  else if(hyperbolic || in_h2xe()) return -intval(h, Hypc);
  #if MAXMDIM >= 4
  else if(sl2) return h[2]*h[2] + h[3]*h[3] - h[0]*h[0] - h[1]*h[1];
  #endif
  else return h[LDIM];
  }

EX int safe_classify_ideals(hyperpoint h) {
  if(hyperbolic || in_h2xe()) {
    h /= h[LDIM];
    ld x = MDIM == 3 ? 1 - (h[0] * h[0] + h[1] * h[1]) : 1 - (h[0] * h[0] + h[1] * h[1] + h[2] * h[2]);
    if(x > 1e-6) return 1;
    if(x < -1e-6) return -1;
    return 0;
    }
  return 1;
  }

EX ld ideal_limit = 10;
EX ld ideal_each = degree;

EX hyperpoint safe_approximation_of_ideal(hyperpoint h) {
  return towards_inf(C0, h, ideal_limit);
  }

/** the point on the line ab which is closest to zero. Might not be normalized. Works even if a and b are (ultra)ideal */
EX hyperpoint closest_to_zero(hyperpoint a, hyperpoint b) {
  if(sqhypot_d(MDIM, a-b) < 1e-9) return a;
  if(isnan(a[0])) return a;
  a /= a[LDIM];
  b /= b[LDIM];
  ld mul_a = 0, mul_b = 0;
  for(int i=0; i<LDIM; i++) {
    ld z = a[i] - b[i];
    mul_a += a[i] * z;
    mul_b -= b[i] * z;
    }

  return (mul_b * a + mul_a * b) / (mul_a + mul_b);
  }

/** should be called get_lof */
EX ld zlevel(const hyperpoint &h) {
  if(sl2) return sqrt(-intval(h, Hypc));
  else if(translatable) return h[LDIM];
  else if(sphere) return sqrt(intval(h, Hypc));
  else if(in_e2xe()) return log(h[2]);
  else if(gproduct) return log(sqrt(abs(intval(h, Hypc)))); /* abs works with both underlying spherical and hyperbolic */
  else return (h[LDIM] < 0 ? -1 : 1) * sqrt(-intval(h, Hypc));
  }

EX ld hypot_auto(ld x, ld y) {
  switch(cgclass) {
    case gcEuclid:
      return hypot(x, y);
    case gcHyperbolic:
      return acosh(cosh(x) * cosh(y));
    case gcSphere:
      return acos(cos(x) * cos(y));
    default:
      return hypot(x, y);
    }
  }

/** normalize the homogeneous coordinates */
EX hyperpoint normalize(hyperpoint H) {
  if(gproduct) return H;
  ld Z = zlevel(H);
  for(int c=0; c<MXDIM; c++) H[c] /= Z;
  return H;
  }

/** like normalize but makes (ultra)ideal points material */
EX hyperpoint ultra_normalize(hyperpoint H) {
  if(material(H) <= 0) {
    H[LDIM] = hypot_d(LDIM, H) + 1e-10;
    }
  return normalize(H);
  }

/** get the center of the line segment from H1 to H2 */
EX hyperpoint mid(const hyperpoint& H1, const hyperpoint& H2) {
  if(gproduct) {
    auto d1 = product_decompose(H1);
    auto d2 = product_decompose(H2);
    hyperpoint res1 = PIU( mid(d1.second, d2.second) );
    hyperpoint res = res1 * exp((d1.first + d2.first) / 2);
    return res;
    }
  return normalize(H1 + H2);
  }

EX shiftpoint mid(const shiftpoint& H1, const shiftpoint& H2) {
  return shiftless(mid(H1.h, H2.h), (H1.shift + H2.shift)/2);
  }

/** like mid, but take 3D into account */
EX hyperpoint midz(const hyperpoint& H1, const hyperpoint& H2) {
  if(gproduct) return mid(H1, H2);
  hyperpoint H3 = H1 + H2;
  
  ld Z = 2;
  
  if(!euclid) Z = zlevel(H3) * 2 / (zlevel(H1) + zlevel(H2));
  for(int c=0; c<MXDIM; c++) H3[c] /= Z;
  
  return H3;
  }

// matrices
//==========

/** rotate by alpha degrees in the coordinates a, b */
EX transmatrix cspin(int a, int b, ld alpha) {
  transmatrix T = Id;
  T[a][a] = +cos(alpha); T[a][b] = +sin(alpha);
  T[b][a] = -sin(alpha); T[b][b] = +cos(alpha);
  return T;
  }

EX transmatrix lorentz(int a, int b, ld v) {
  transmatrix T = Id;
  T[a][a] = T[b][b] = cosh(v);
  T[a][b] = T[b][a] = sinh(v);
  return T;
  }

/** rotate by 90 degrees in the coordinates a, b */
EX transmatrix cspin90(int a, int b) {
  transmatrix T = Id;
  T[a][a] = 0; T[a][b] = 1;
  T[b][a] = -1; T[b][b] = 0;
  return T;
  }

/** rotate by 180 degrees in the coordinates a, b */
EX transmatrix cspin180(int a, int b) {
  transmatrix T = Id;
  T[a][a] = T[b][b] = -1;
  return T;
  }

EX transmatrix random_spin3() {
  ld alpha2 = asin(randd() * 2 - 1);
  ld alpha = randd() * TAU;
  ld alpha3 = randd() * TAU;
  return cspin(0, 1, alpha) * cspin(0, 2, alpha2) * cspin(1, 2, alpha3);
  }

EX transmatrix random_spin() {
  if(WDIM == 2) return spin(randd() * TAU);
  else return random_spin3();
  }

EX transmatrix eupush(ld x, ld y) {
  transmatrix T = Id;
  T[0][LDIM] = x;
  T[1][LDIM] = y;
  return T;
  }

EX transmatrix euclidean_translate(ld x, ld y, ld z) {
  transmatrix T = Id;
  T[0][LDIM] = x;
  T[1][LDIM] = y;
  T[2][LDIM] = z;
  return T;
  }

EX transmatrix euscale(ld x, ld y) {
  transmatrix T = Id;
  T[0][0] = x;
  T[1][1] = y;
  return T;
  }

EX transmatrix euscale3(ld x, ld y, ld z) {
  transmatrix T = Id;
  T[0][0] = x;
  T[1][1] = y;
  T[2][2] = z;
  return T;
  }

EX transmatrix eupush(hyperpoint h, ld co IS(1)) {
  if(nonisotropic) return nisot::translate(h, co);
  if(hyperbolic) { return co ? parabolic13_at(deparabolic13(h)) : inverse(parabolic13_at(deparabolic13(h))); }
  transmatrix T = Id;
  for(int i=0; i<GDIM; i++) T[i][LDIM] = h[i] * co;
  return T;
  }

EX transmatrix eupush3(ld x, ld y, ld z) {
  if(sl2) return slr::translate(slr::xyz_point(x, y, z));
  return eupush(point3(x, y, z));
  }

EX transmatrix euscalezoom(hyperpoint h) {
  transmatrix T = Id;
  T[0][0] = h[0];
  T[0][1] = -h[1];
  T[1][0] = h[1];
  T[1][1] = h[0];
  return T;
  }

EX transmatrix euaffine(hyperpoint h) {
  transmatrix T = Id;
  T[0][1] = h[0];
  T[1][1] = exp(h[1]);
  return T;
  }

EX transmatrix cpush(int cid, ld alpha) {
  if(gproduct && cid == 2)
    return scale_matrix(Id, exp(alpha));
  transmatrix T = Id;
  if(nonisotropic) 
    return eupush3(cid == 0 ? alpha : 0, cid == 1 ? alpha : 0, cid == 2 ? alpha : 0);
  T[LDIM][LDIM] = T[cid][cid] = cos_auto(alpha);
  T[cid][LDIM] = sin_auto(alpha);
  T[LDIM][cid] = -curvature() * sin_auto(alpha);
  return T;
  }

EX transmatrix cmirror(int cid) {
  transmatrix T = Id;
  T[cid][cid] = -1;
  return T;
  }

// push alpha units to the right
EX transmatrix xpush(ld alpha) { return cpush(0, alpha); }

EX bool eqmatrix(transmatrix A, transmatrix B, ld eps IS(.01)) {
  for(int i=0; i<MXDIM; i++)
  for(int j=0; j<MXDIM; j++)
    if(std::abs(A[i][j] - B[i][j]) > eps)
      return false;
  return true;
  }

// push alpha units vertically
EX transmatrix ypush(ld alpha) { return cpush(1, alpha); }

EX transmatrix zpush(ld z) { return cpush(2, z); }

EX transmatrix matrix3(ld a, ld b, ld c, ld d, ld e, ld f, ld g, ld h, ld i) {
  #if MAXMDIM==3
  return transmatrix {{{a,b,c},{d,e,f},{g,h,i}}};
  #else
  if(GDIM == 2 || MDIM == 3)
    return transmatrix {{{a,b,c,0},{d,e,f,0},{g,h,i,0},{0,0,0,1}}};
  else
    return transmatrix {{{a,b,0,c},{d,e,0,f},{0,0,1,0},{g,h,0,i}}};
  #endif
  }

EX transmatrix matrix4(ld a, ld b, ld c, ld d, ld e, ld f, ld g, ld h, ld i, ld j, ld k, ld l, ld m, ld n, ld o, ld p) {
  #if MAXMDIM==3
  return transmatrix {{{a,b,d},{e,f,h},{m,n,p}}};
  #else
  return transmatrix {{{a,b,c,d},{e,f,g,h},{i,j,k,l},{m,n,o,p}}};
  #endif
  }

EX transmatrix parabolic1(ld u) {
  if(euclid)
    return ypush(u);
  else if(cgi.emb->is_hyp_in_solnih() && !geom3::flipped) {
    return ypush(u);
    }
  else {
    ld diag = u*u/2;
    return matrix3(
      -diag+1, u, diag,
      -u, 1, u,
      -diag, u, diag+1
      );
    }
  }

EX transmatrix parabolic13(ld u, ld v) {
  if(euclid)
    return eupush3(0, u, v);
  else if(cgi.emb->is_euc_in_hyp()) {
    ld diag = (u*u+v*v)/2;
    return matrix4(
      1, 0, -u, u,
      0, 1, -v, v,
      u, v, -diag+1, diag,
      u, v, -diag, diag+1
      );
    }
  else {
    ld diag = (u*u+v*v)/2;
    return matrix4(
      -diag+1, u, v, diag,
      -u, 1, 0, u,
      -v, 0, 1, v,
      -diag, u, v, diag+1
      );
    }
  }

EX hyperpoint kleinize(hyperpoint h) {
  #if MAXMDIM == 3
  return point3(h[0]/h[2], h[1]/h[2], 1);
  #else
  if(GDIM == 2) return point3(h[0]/h[2], h[1]/h[2], 1);
  else return point31(h[0]/h[3], h[1]/h[3], h[2]/h[3]);
  #endif
  }

EX hyperpoint deparabolic13(hyperpoint h) {
  if(euclid) return h;
  if(cgi.emb->is_euc_in_hyp()) {
    h /= (1 + h[LDIM]);
    h[2] -= 1;
    h /= sqhypot_d(LDIM, h);
    h[2] += .5;
    return point3(h[0] * 2, h[1] * 2, log(2) + log(-h[2]));
    }
  h /= (1 + h[LDIM]);
  h[0] -= 1;
  h /= sqhypot_d(LDIM, h);
  h[0] += .5;
  return point3(log(2) + log(-h[0]), h[1] * 2, LDIM==3 ? h[2] * 2 : 0);
  }

EX hyperpoint parabolic13(hyperpoint h) {
  if(euclid) return h;
  else if(cgi.emb->is_euc_in_hyp()) {
    return parabolic13(h[0], h[1]) * cpush0(2, h[2]);
    }
  else if(LDIM == 3)
    return parabolic13(h[1], h[2]) * xpush0(h[0]);
  else
    return parabolic1(h[1]) * xpush0(h[0]);
  }

EX transmatrix parabolic13_at(hyperpoint h) {
  if(euclid) return rgpushxto0(h);
  else if(cgi.emb->is_euc_in_hyp()) {
    return parabolic13(h[0], h[1]) * cpush(2, h[2]);
    }
  else if(LDIM == 3)
    return parabolic13(h[1], h[2]) * xpush(h[0]);
  else
    return parabolic1(h[1]) * xpush(h[0]);
  }

EX transmatrix spintoc(const hyperpoint& H, int t, int f) {
  transmatrix T = Id;
  ld R = hypot(H[f], H[t]);
  if(R >= 1e-15) {
    T[t][t] = +H[t]/R; T[t][f] = +H[f]/R;
    T[f][t] = -H[f]/R; T[f][f] = +H[t]/R;
    }
  return T;
  }

/** an Euclidean rotation in the axes (t,f) which rotates
 *  the point H to the positive 't' axis 
 */

EX transmatrix rspintoc(const hyperpoint& H, int t, int f) {
  transmatrix T = Id;
  ld R = hypot(H[f], H[t]);
  if(R >= 1e-15) {
    T[t][t] = +H[t]/R; T[t][f] = -H[f]/R;
    T[f][t] = +H[f]/R; T[f][f] = +H[t]/R;
    }
  return T;
  }

/** an isometry which takes the point H to the positive X axis 
 *  \see rspintox 
 */
EX transmatrix spintox(const hyperpoint& H) {
  if(GDIM == 2 || gproduct) return spintoc(H, 0, 1);
  transmatrix T1 = spintoc(H, 0, 1);
  return spintoc(T1*H, 0, 2) * T1;
  }

/** inverse of hr::spintox 
 */
EX transmatrix rspintox(const hyperpoint& H) {
  if(GDIM == 2 || gproduct) return rspintoc(H, 0, 1);
  transmatrix T1 = spintoc(H, 0, 1);
  return rspintoc(H, 0, 1) * rspintoc(T1*H, 0, 2);
  }

/** for H on the X axis, this matrix pushes H to C0 
 *  \see gpushxto0
 */

EX transmatrix pushxto0(const hyperpoint& H) {
  transmatrix T = Id;
  T[0][0] = +H[LDIM]; T[0][LDIM] = -H[0];
  T[LDIM][0] = curvature() * H[0]; T[LDIM][LDIM] = +H[LDIM];
  return T;
  }

/** set the i-th column of T to H */
EX void set_column(transmatrix& T, int i, const hyperpoint& H) {
  for(int j=0; j<MXDIM; j++)
    T[j][i] = H[j];
  }

EX hyperpoint get_column(transmatrix& T, int i) {
  hyperpoint h;
  for(int j=0; j<MXDIM; j++)
    h[j] = T[j][i];
  return h;
  }

/** build a matrix using the given vectors as columns */
EX transmatrix build_matrix(hyperpoint h1, hyperpoint h2, hyperpoint h3, hyperpoint h4) {
  transmatrix T;
  for(int i=0; i<MXDIM; i++) {
    T[i][0] = h1[i],
    T[i][1] = h2[i],
    T[i][2] = h3[i];
    if(MAXMDIM == 4) T[i][3] = h4[i];
    }
  return T;
  }

/** for H on the X axis, this matrix pushes C0 to H
 *  \see rgpushxto0
 */

EX transmatrix rpushxto0(const hyperpoint& H) {
  transmatrix T = Id;
  T[0][0] = +H[LDIM]; T[0][LDIM] = H[0];
  T[LDIM][0] = -curvature() * H[0]; T[LDIM][LDIM] = +H[LDIM];
  return T;
  }

EX transmatrix ggpushxto0(const hyperpoint& H, ld co) {
  if(translatable) 
    return eupush(H, co);
  if(gproduct) {
    auto d = product_decompose(H);
    return scale_matrix(PIU(ggpushxto0(d.second, co)), exp(d.first * co));
    }
  transmatrix res = Id;
  if(sqhypot_d(GDIM, H) < 1e-16) return res;
  ld fac = -curvature()/(H[LDIM]+1);
  for(int i=0; i<GDIM; i++)
  for(int j=0; j<GDIM; j++)
    res[i][j] += H[i] * H[j] * fac;
    
  for(int d=0; d<GDIM; d++)
    res[d][LDIM] = co * H[d],
    res[LDIM][d] = -curvature() * co * H[d];
  res[LDIM][LDIM] = H[LDIM];
  
  return res;
  }

/** a translation matrix which takes H to 0 */
EX transmatrix gpushxto0(const hyperpoint& H) {
  return ggpushxto0(H, -1);
  }

/** a translation matrix which takes 0 to H */
EX transmatrix rgpushxto0(const hyperpoint& H) {
  return ggpushxto0(H, 1);
  }

EX shiftmatrix rgpushxto0(const shiftpoint& H) {
  return shiftless(rgpushxto0(H.h), H.shift);
  }

EX bool dont_fixmatrix;

/** \brief Fix the numerical inaccuracies in the isometry T
 *
 *  The nature of hyperbolic geometry makes the computations numerically unstable.
 *  The numerical errors tend to accumulate, eventually destroying the projection.
 *  This function fixes this problem by replacing T with a 'correct' isometry.
 */

EX void fixmatrix(transmatrix& T) {
  if(dont_fixmatrix) return;
  else if(nonisotropic) ; // T may be inverse... do not do that
  else if(cgflags & qAFFINE) ; // affine
  else if(gproduct) {
    auto z = zlevel(tC0(T));
    T = scale_matrix(T, exp(-z));
    PIU(fixmatrix(T));
    T = scale_matrix(T, exp(+z));
    }
  else if(euclid)
    fixmatrix_euclid(T);
  else
    orthonormalize(T);
  }

EX void fixmatrix_euclid(transmatrix& T) {
  for(int x=0; x<GDIM; x++) for(int y=0; y<=x; y++) {
    ld dp = 0;
    for(int z=0; z<GDIM; z++) dp += T[z][x] * T[z][y];
    
    if(y == x) dp = 1 - sqrt(1/dp);
    
    for(int z=0; z<GDIM; z++) T[z][x] -= dp * T[z][y];
    }
  for(int x=0; x<GDIM; x++) T[LDIM][x] = 0;
  T[LDIM][LDIM] = 1;
  }

EX void orthonormalize(transmatrix& T) {
  for(int x=0; x<MDIM; x++) for(int y=0; y<=x; y++) {
    ld dp = 0;
    for(int z=0; z<MXDIM; z++) dp += T[z][x] * T[z][y] * sig(z);
    
    if(y == x) dp = 1 - sqrt(sig(x)/dp);
    
    for(int z=0; z<MXDIM; z++) T[z][x] -= dp * T[z][y];
    }
  }

/** fix a 3D rotation matrix */
EX void fix_rotation(transmatrix& rot) {
  dynamicval<eGeometry> g(geometry, gSphere); 
  fixmatrix(rot); 
  fix4(rot);
  }

/** determinant 2x2 */
EX ld det2(const transmatrix& T) {
  return T[0][0] * T[1][1] - T[0][1] * T[1][0];
  }

/** determinant 3x3 */
EX ld det3(const transmatrix& T) {
  ld det = 0;
  for(int i=0; i<3; i++) 
    det += T[0][i] * T[1][(i+1)%3] * T[2][(i+2)%3];
  for(int i=0; i<3; i++) 
    det -= T[0][i] * T[1][(i+2)%3] * T[2][(i+1)%3];
  return det;
  }

/** determinant */
EX ld det(const transmatrix& T) {
  if(MDIM == 3) 
    return det3(T);
  else {
    ld det = 1;
    transmatrix M = T;
    for(int a=0; a<MDIM; a++) {
      int max_at = a;
      for(int b=a; b<MDIM; b++)
        if(abs(M[b][a]) > abs(M[max_at][a]))
          max_at = b;
      
      if(max_at != a)
        for(int c=a; c<MDIM; c++) tie(M[max_at][c], M[a][c]) = make_pair(-M[a][c], M[max_at][c]);

      if(!M[a][a]) return 0;
      for(int b=a+1; b<MDIM; b++) {
        ld co = -M[b][a] / M[a][a];
        for(int c=a; c<MDIM; c++) M[b][c] += M[a][c] * co;
        }
      det *= M[a][a];
      }
    return det;
    }
  }

/** warning about incorrect inverse */
void inverse_error(const transmatrix& T) {
  println(hlog, "Warning: inverting a singular matrix: ", T);
  }

/** inverse of a 3x3 matrix */
EX transmatrix inverse3(const transmatrix& T) {
  ld d = det3(T);
  transmatrix T2;
  if(d == 0) {
    inverse_error(T);
    return Id;
    }

  for(int i=0; i<3; i++)
  for(int j=0; j<3; j++)
    T2[j][i] = (T[(i+1)%3][(j+1)%3] * T[(i+2)%3][(j+2)%3] - T[(i+1)%3][(j+2)%3] * T[(i+2)%3][(j+1)%3]) / d;
  return T2;
  }

/** inverse of a 2x2 matrix */
EX transmatrix inverse2(const transmatrix& T) {
  ld d = det2(T);
  if(d == 0) {
    inverse_error(T);
    return Id;
    }

  transmatrix T2 = Id;
  T2[0][0] = T[1][1] / d;
  T2[1][1] = T[0][0] / d;
  T2[0][1] = -T[0][1] / d;
  T2[1][0] = -T[1][0] / d;
  return T2;
  }

/** \brief inverse of a general matrix */
EX transmatrix inverse(const transmatrix& T) {
  if(MDIM == 3) 
    return inverse3(T);
  else {
    transmatrix T1 = T;
    transmatrix T2 = Id;
  
    for(int a=0; a<MDIM; a++) {
      int best = a;
      
      for(int b=a+1; b<MDIM; b++)
        if(abs(T1[b][a]) > abs(T1[best][a]))
          best = b;

      int b = best;

      if(b != a)
        for(int c=0; c<MDIM; c++) 
          swap(T1[b][c], T1[a][c]), swap(T2[b][c], T2[a][c]);

      if(!T1[a][a]) { inverse_error(T); return Id; }
      for(int b=a+1; b<=GDIM; b++) {
        ld co = -T1[b][a] / T1[a][a];
        for(int c=0; c<MDIM; c++) T1[b][c] += T1[a][c] * co, T2[b][c] += T2[a][c] * co;
        }
      }
    
    for(int a=MDIM-1; a>=0; a--) {
      for(int b=0; b<a; b++) {
        ld co = -T1[b][a] / T1[a][a];
        for(int c=0; c<MDIM; c++) T1[b][c] += T1[a][c] * co, T2[b][c] += T2[a][c] * co;
        }
      ld co = 1 / T1[a][a];
      for(int c=0; c<MDIM; c++) T1[a][c] *= co, T2[a][c] *= co;
      }
    return T2;
    }
  }

/** \brief inverse of an orthogonal matrix, i.e., transposition */
EX transmatrix ortho_inverse(transmatrix T) {
  for(int i=1; i<MDIM; i++)
    for(int j=0; j<i; j++)
      swap(T[i][j], T[j][i]);
  return T;
  }

/** \brief inverse of an orthogonal matrix in Minkowski space */
EX transmatrix pseudo_ortho_inverse(transmatrix T) {
  for(int i=1; i<MXDIM; i++)
    for(int j=0; j<i; j++)
      swap(T[i][j], T[j][i]);
  for(int i=0; i<MDIM-1; i++)
    T[i][MDIM-1] = -T[i][MDIM-1],
    T[MDIM-1][i] = -T[MDIM-1][i];
  return T;
  }

/** \brief inverse of an isometry -- in most geometries this can be done more efficiently than using inverse */
EX transmatrix iso_inverse(const transmatrix& T) {
  if(hyperbolic)
    return pseudo_ortho_inverse(T);
  if(sphere) 
    return ortho_inverse(T);
  if(nil && nilv::model_used == 1) {
    transmatrix U = Id;    
    U[2][LDIM] = T[0][LDIM] * T[1][LDIM] - T[2][LDIM];
    U[1][LDIM] = -T[1][LDIM];
    U[2][1] = U[0][LDIM] = -T[0][LDIM];
    return U;
    }
  if(euclid && !(cgflags & qAFFINE)) {
    transmatrix U = Id;
    for(int i=0; i<MDIM-1; i++)
      for(int j=0; j<MDIM-1; j++)
        U[i][j] = T[j][i];
    hyperpoint h = U * tC0(T);
    for(int i=0; i<MDIM-1; i++)
      U[i][MDIM-1] = -h[i];
    return U;
    }
  return inverse(T);
  }

/** inverse a guaranteed rotation */
EX transmatrix rot_inverse(const transmatrix& T) {
  return transpose(T);
  }

/** inverse a guaranteed rotation */
EX trans23 rot_inverse(const trans23& T) {
  return trans23(rot_inverse(T.v2), rot_inverse(T.v3));
  }

/** \brief T inverse a matrix T = O*S, where O is isometry and S is a scaling matrix (todo optimize) */
EX transmatrix z_inverse(const transmatrix& T) {
  return inverse(T);
  }

/** \brief T inverse a matrix T = O*P, where O is orthogonal and P is an isometry (todo optimize) */
EX transmatrix view_inverse(transmatrix T) {
  if(nonisotropic) return inverse(T);
  if(gproduct) return z_inverse(T);
  return iso_inverse(T);
  }

/** \brief T inverse a matrix T = P*O, where O is orthogonal and P is an isometry (todo optimize) */
EX transmatrix iview_inverse(transmatrix T) {
  if(nonisotropic) return inverse(T);
  if(gproduct) return z_inverse(T);
  return iso_inverse(T);
  }

EX pair<ld, hyperpoint> product_decompose(hyperpoint h) {
  ld z = zlevel(h);
  return make_pair(z, scale_point(h, exp(-z)));
  }

/** distance from mh and 0 */
EX ld hdist0(const hyperpoint& mh) {
  switch(cgclass) {
    case gcHyperbolic:
      if(mh[LDIM] < 1) return 0;
      return acosh(mh[LDIM]);
    case gcEuclid: {
      return hypot_d(GDIM, mh);
      }
    case gcSphere: {
      ld res = mh[LDIM] >= 1 ? 0 : mh[LDIM] <= -1 ? M_PI : acos(mh[LDIM]);
      return res;
      }
    case gcProduct: {
      auto d1 = product_decompose(mh);
      return hypot(PIU(hdist0(d1.second)), d1.first);
      }
    #if MAXMDIM >= 4
    case gcSL2: {
      auto cosh_r = hypot(mh[2], mh[3]);
      auto phi = atan2(mh[2], mh[3]);
      return hypot(cosh_r < 1 ? 0 : acosh(cosh_r), phi);
      }
    case gcNil: {
      ld bz = mh[0] * mh[1] / 2;
      return hypot(mh[0], mh[1]) + abs(mh[2] - bz);
      }
    #endif
    default:
      return hypot_d(GDIM, mh);
    }
  }

EX ld hdist0(const shiftpoint& mh) {
  return hdist0(unshift(mh));
  }

/** length of a circle of radius r */
EX ld circlelength(ld r) {
  switch(cgclass) {
    case gcEuclid:
      return TAU * r;
    case gcHyperbolic:
      return TAU * sinh(r);
    case gcSphere:
      return TAU * sin(r);
    default:
      return TAU * r;
    }
  }

/* distance between h1 and h2 */
EX ld hdist(const hyperpoint& h1, const hyperpoint& h2) {
  ld iv = intval(h1, h2);
  switch(cgclass) {
    case gcEuclid:
      if(iv < 0) return 0;
      return sqrt(iv);
    case gcHyperbolic:
      if(iv < 0) return 0;
      return 2 * asinh(sqrt(iv) / 2);
    case gcSphere:
      return 2 * asin_auto_clamp(sqrt(iv) / 2);
    case gcProduct: {
      auto d1 = product_decompose(h1);
      auto d2 = product_decompose(h2);
      return hypot(PIU(hdist(d1.second, d2.second)), d1.first - d2.first);
      }
    case gcSL2: 
      return hdist0(stretch::itranslate(h1) * h2);
    default:
      if(iv < 0) return 0;
      return sqrt(iv);
    }
  }

EX ld hdist(const shiftpoint& h1, const shiftpoint& h2) {
  return hdist(h1.h, unshift(h2, h1.shift));
  }

/** like orthogonal_move but fol may be factor (in 2D graphics) or level (elsewhere) */
EX hyperpoint orthogonal_move_fol(const hyperpoint& h, double fol) {
  if(GDIM == 2) return scale_point(h, fol);
  else return orthogonal_move(h, fol);
  }

/** like orthogonal_move but fol may be factor (in 2D graphics) or level (elsewhere) */
EX transmatrix orthogonal_move_fol(const transmatrix& T, double fol) {
  if(GDIM == 2) return scale_matrix(T, fol);
  else return orthogonal_move(T, fol);
  }

/** like orthogonal_move but fol may be factor (in 2D graphics) or level (elsewhere) */
EX shiftmatrix orthogonal_move_fol(const shiftmatrix& T, double fol) {
  if(GDIM == 2) return scale_matrix(T, fol);
  else return orthogonal_move(T, fol);
  }

/** the scaling matrix (Euclidean homogeneous scaling; also shift by log(scale) in product space */
EX transmatrix scale_matrix(const transmatrix& t, ld scale_factor) {
  transmatrix res;
  for(int i=0; i<MXDIM; i++) {
    for(int j=0; j<MDIM; j++)
      res[i][j] = t[i][j] * scale_factor;
    for(int j=MDIM; j<MXDIM; j++)
      res[i][j] = t[i][j];
    }
  return res;
  }

/** the scaling matrix (Euclidean homogeneous scaling; also shift by log(scale) in product space */
EX shiftmatrix scale_matrix(const shiftmatrix& t, ld scale_factor) {
  return shiftless(scale_matrix(t.T, scale_factor), t.shift);
  }

/** the scaling matrix (Euclidean homogeneous scaling; also shift by log(scale) in product space */
EX hyperpoint scale_point(const hyperpoint& h, ld scale_factor) {
  hyperpoint res;
  for(int j=0; j<MDIM; j++)
    res[j] = h[j] * scale_factor;
  for(int j=MDIM; j<MXDIM; j++)
    res[j] = h[j];
  return res;
  }

EX transmatrix orthogonal_move(const transmatrix& t, double level) {
  if(GDIM == 3) return t * lzpush(level);
  return scale_matrix(t, geom3::lev_to_factor(level));
  }

EX shiftmatrix orthogonal_move(const shiftmatrix& t, double level) {
  return shiftless(orthogonal_move(t.T, level), t.shift);
  }

/** fix a 3x3 matrix into a 4x4 matrix, in place */
EX void fix4(transmatrix& t) {
  #if MAXMDIM > 3
  if(ldebug) println(hlog, "fix4 performed");
  for(int i=0; i<4; i++) t[3][i] = t[i][3] = i == 3;
  #endif
  }

/** fix a 3x3 matrix into a 4x4 matrix, as a function */
EX transmatrix fix4_f(transmatrix t) { fix4(t); return t; }

EX transmatrix xyscale(const transmatrix& t, double fac) {
  transmatrix res;
  for(int i=0; i<MXDIM; i++) {
    for(int j=0; j<GDIM; j++)
      res[i][j] = t[i][j] * fac;
    for(int j=GDIM; j<MXDIM; j++)
      res[i][j] = t[i][j];
    }
  return res;
  }

EX transmatrix xyzscale(const transmatrix& t, double fac, double facz) {
  transmatrix res;
  for(int i=0; i<MXDIM; i++) {
    for(int j=0; j<GDIM; j++)
      res[i][j] = t[i][j] * fac;
    res[i][LDIM] = 
      t[i][LDIM] * facz;
    for(int j=LDIM+1; j<MXDIM; j++)
      res[i][j] = t[i][j];
    }
  return res;
  }

EX shiftmatrix xyzscale(const shiftmatrix& t, double fac, double facz) {
  return shiftless(xyzscale(t.T, fac, facz), t.shift);
  }

EX transmatrix mzscale(const transmatrix& t, double fac) {
  if(GDIM == 3) return t * cpush(2, fac);
  // take only the spin
  transmatrix tcentered = gpushxto0(tC0(t)) * t;
  // tcentered = tcentered * spin(downspin_zivory);
  fac -= 1;
  transmatrix res = t * inverse(tcentered) * ypush(-fac) * tcentered;
  fac *= .2;
  fac += 1;
  for(int i=0; i<MXDIM; i++) for(int j=0; j<MXDIM; j++)
    res[i][j] = res[i][j] * fac;
  return res;
  }

EX shiftmatrix mzscale(const shiftmatrix& t, double fac) {
  return shiftless(mzscale(t.T, fac), t.shift);
  }

EX hyperpoint mid3(hyperpoint h1, hyperpoint h2, hyperpoint h3) {
  return mid(h1+h2+h3, h1+h2+h3);
  }

EX hyperpoint mid_at(hyperpoint h1, hyperpoint h2, ld v) {
  hyperpoint h = h1 * (1-v) + h2 * v;
  return mid(h, h);
  }  

EX hyperpoint mid_at_actual(hyperpoint h, ld v) {
  return rspintox(h) * xpush0(hdist0(h) * v);
  }

#if MAXMDIM >= 4
/** in 3D, an orthogonal projection of C0 on the given triangle */
EX hyperpoint orthogonal_of_C0(hyperpoint h0, hyperpoint h1, hyperpoint h2) {
  h0 /= h0[3];
  h1 /= h1[3];
  h2 /= h2[3];
  hyperpoint w = h0;
  hyperpoint d1 = h1 - h0;
  hyperpoint d2 = h2 - h0;
  ld denom = (d1|d1) * (d2|d2) - (d1|d2) * (d1|d2);
  ld a1 = (d2|w) * (d1|d2) - (d1|w) * (d2|d2);
  ld a2 = (d1|w) * (d1|d2) - (d2|w) * (d1|d1);
  hyperpoint h = w * denom + d1 * a1 + d2 * a2;
  return normalize(h);
  }
#endif

EX hyperpoint hpxd(ld d, ld x, ld y, ld z) {
  hyperpoint H = hpxyz(d*x, d*y, z);
  H = mid(H, H);
  return H;
  }

EX ld signum(ld x) { return x<0?-1:x>0?1:0; }

EX bool asign(ld y1, ld y2) { return signum(y1) != signum(y2); }

EX ld xcross(ld x1, ld y1, ld x2, ld y2) { return x1 + (x2 - x1) * y1 / (y1 - y2); }

#if HDR
enum eShiftMethod { smProduct, smIsotropic, smEmbedded, smLie, smGeodesic };
enum eEmbeddedShiftMethodChoice { smcNone, smcBoth, smcAuto };
enum eShiftMethodApplication { smaManualCamera, smaAutocenter, smaObject, smaWallRadar, smaAnimation };
#endif

EX eEmbeddedShiftMethodChoice embedded_shift_method_choice = smcBoth;

EX bool use_embedded_shift(eShiftMethodApplication sma) {
  switch(sma) {
    case smaAutocenter: case smaAnimation: return embedded_shift_method_choice != smcNone;
    case smaManualCamera: return embedded_shift_method_choice == smcBoth;
    case smaObject: return true;
    case smaWallRadar: return among(pmodel, mdLiePerspective, mdLieOrthogonal);
    default: throw hr_exception("unknown sma");
    }
  }

EX eShiftMethod shift_method(eShiftMethodApplication sma) {
  if(gproduct) return smProduct;
  if(embedded_plane && sma == smaObject) return smEmbedded;
  if(embedded_plane && use_embedded_shift(sma)) return smEmbedded;
  if(!nonisotropic && !stretch::in() && !(!nisot::geodesic_movement && hyperbolic && bt::in())) return smIsotropic;
  if(!nisot::geodesic_movement && !embedded_plane) return smLie;
  return smGeodesic;
  }

EX transmatrix shift_object(transmatrix Position, const transmatrix& ori, const hyperpoint direction, eShiftMethod sm IS(shift_method(smaObject))) {
  switch(sm) {
    case smGeodesic:
      return nisot::parallel_transport(Position, direction);
    case smLie:
      return nisot::lie_transport(Position, direction);
    case smProduct: {
      hyperpoint h = product::direct_exp(ori * direction);
      return Position * rgpushxto0(h);
      }
    case smIsotropic: {
      return Position * rgpushxto0(direct_exp(direction));
      }
    case smEmbedded: {

      // optimization -- should work without it
      if(cgi.emb->is_same_in_same()) return Position * rgpushxto0(direct_exp(direction));

      if(gproduct) {
        return Position * cgi.emb->intermediate_to_actual_translation(ori * cgi.emb->logical_to_intermediate * direction);
        }

      auto P = inverse_shift(ggmatrix(cwt.at), shiftless(Position));

      hyperpoint a = P * tile_center();
      hyperpoint i0 = cgi.emb->actual_to_intermediate(a);
      auto l0 = cgi.emb->intermediate_to_logical * i0;
      auto l = l0; l[2] = 0;
      auto i = cgi.emb->logical_to_intermediate * l;
      auto rot1 = inverse(cgi.emb->intermediate_to_actual_translation(i)) * P;
      auto rot = cgi.emb->intermediate_to_logical_scaled * rot1 * cgi.emb->logical_scaled_to_intermediate;

      auto par = cgi.emb->logical_to_intermediate * rot * direction;
      if(cgi.emb->is_sph_in_low()) par = cspin180(0, 1) * par; // reason unknown
      auto tra = cgi.emb->intermediate_to_actual_translation(par);
      return Position * inverse(rot1) * tra * rot1;
      }
    default: throw hr_exception("unknown shift method in shift_object");
    }
  }

EX void apply_shift_object(transmatrix& Position, const transmatrix orientation, const hyperpoint direction, eShiftMethod sm IS(shift_method(smaObject))) {
  Position = shift_object(Position, orientation, direction, sm);
  }

EX void rotate_object(transmatrix& Position, transmatrix& orientation, transmatrix R) {
  if(cgi.emb->is_euc_in_product()) orientation = orientation * R;
  else if(gproduct && WDIM == 3) orientation = orientation * R;
  else Position = Position * R;
  }

EX transmatrix spin_towards(const transmatrix Position, transmatrix& ori, const hyperpoint goal, int dir, int back) {
  transmatrix T;
  ld alpha = 0;
  if(nonisotropic && nisot::geodesic_movement)
    T = nisot::spin_towards(Position, goal);
  else { 
    hyperpoint U = inverse(Position) * goal;
    if(gproduct) {
      hyperpoint h = product::inverse_exp(U);
      alpha = asin_clamp(h[2] / hypot_d(3, h));
      U = product_decompose(U).second;
      }
    T = rspintox(U);
    }
  if(back < 0) T = T * spin180(), alpha = -alpha;
  if(gproduct) {
    if(dir == 0) ori = cspin(2, 0, alpha);
    if(dir == 2) ori = cspin(2, 0, alpha - 90._deg), dir = 0;
    }
  if(dir) T = T * cspin(dir, 0, -90._deg);
  T = Position * T;
  return T;
  }

EX shiftmatrix spin_towards(const shiftmatrix Position, transmatrix& ori, const shiftpoint goal, int dir, int back) {
  return shiftless(spin_towards(Position.T, ori, unshift(goal, Position.shift), dir, back), Position.shift);
  }

EX ld ortho_error(transmatrix T) {

  ld err = 0;
  
  for(int x=0; x<3; x++) for(int y=0; y<3; y++) {
      ld s = 0;
      for(int z=0; z<3; z++) s += T[z][x] * T[z][y];
      
      s -= (x==y);
      err += s*s;
      }

  return err;      
  }

EX transmatrix transpose(transmatrix T) {
  transmatrix result;
  for(int i=0; i<MXDIM; i++)
    for(int j=0; j<MXDIM; j++)
      result[j][i] = T[i][j];
  return result;
  }

#if HDR
namespace slr { 
  hyperpoint xyz_point(ld x, ld y, ld z); 
  hyperpoint polar(ld r, ld theta, ld phi); 
  }

inline hyperpoint cpush0(int c, ld x) { 
  hyperpoint h = Hypc;
  if(sl2) return slr::xyz_point(c==0?x:0, c==1?x:0, c==2?x:0);
  if(c == 2 && gproduct) {
    h[2] = exp(x);
    return h;
    }
  h[LDIM] = cos_auto(x);
  h[c] = sin_auto(x);
  return h;
  }

inline hyperpoint xpush0(ld x) { return cpush0(0, x); }
inline hyperpoint ypush0(ld x) { return cpush0(1, x); }
inline hyperpoint zpush0(ld x) { return cpush0(2, x); }

/** T * C0, optimized */
inline hyperpoint tC0(const transmatrix &T) {
  hyperpoint z;
  for(int i=0; i<MXDIM; i++) z[i] = T[i][LDIM];
  return z;
  }

inline hyperpoint tC0_t(const transmatrix &T) { return tC0(T); }

inline shiftpoint tC0(const shiftmatrix &T) {
  return shiftpoint{tC0(T.T), T.shift};
  }
#endif

/** tangent vector in the given direction */
EX hyperpoint ctangent(int c, ld x) { return point3(c==0?x:0, c==1?x:0, c==2?x:0); }

/** tangent vector in direction X */
EX hyperpoint xtangent(ld x) { return ctangent(0, x); }

/** tangent vector in direction Z */
EX hyperpoint ztangent(ld z) { return ctangent(2, z); }

/** change the length of the targent vector */
EX hyperpoint tangent_length(hyperpoint dir, ld length) {
  ld r = hypot_d(GDIM, dir);
  if(!r) return dir;
  return dir * (length / r);
  }

/** exponential function: follow the geodesic given by v */
EX hyperpoint direct_exp(hyperpoint v) {
  #if CAP_SOLV
  if(sn::in()) return nisot::numerical_exp(v);
  #endif
  #if MAXMDIM >= 4
  if(nil) return nilv::formula_exp(v);
  if(sl2 || stretch::in()) return stretch::mstretch ? nisot::numerical_exp(v) : unshift(twist::formula_exp(v));
  #endif
  if(gproduct) return product::direct_exp(v);
  ld d = hypot_d(GDIM, v);
  if(d > 0) for(int i=0; i<GDIM; i++) v[i] = v[i] * sin_auto(d) / d;
  v[LDIM] = cos_auto(d);
  return v;
  }

#if HDR
constexpr flagtype pfNO_INTERPOLATION = 1; /**< in tables (sol/nih geometries), do not use interpolations */
constexpr flagtype pfNO_DISTANCE      = 2; /**< we just need the directions -- this makes it a bit faster in sol/nih geometries */
constexpr flagtype pfLOW_BS_ITER      = 4; /**< low iterations in binary search (nil geometry, sl2 not affected currently) */

constexpr flagtype pQUICK     = pfNO_INTERPOLATION | pfLOW_BS_ITER;

constexpr flagtype pNORMAL    = 0;
#endif
  
/** inverse exponential function \see hr::direct_exp */
EX hyperpoint inverse_exp(const shiftpoint h, flagtype prec IS(pNORMAL)) {
  #if CAP_SOLV
  if(sn::in()) {
    /* this will be more precise for use in set_view in intra */
    if(sqhypot_d(3, h.h) < 2e-9) return h.h - C0;
    if(nih) 
      return sn::get_inverse_exp_nsym(h.h, prec);
    else
      return sn::get_inverse_exp_symsol(h.h, prec);
    }
  #endif
  if(nil) return nilv::get_inverse_exp(h.h, prec);
  if(sl2) return slr::get_inverse_exp(h);
  if(gproduct) return product::inverse_exp(h.h);
  ld d = acos_auto_clamp(h[GDIM]);
  hyperpoint v = Hypc;
  if(d && sin_auto(d)) for(int i=0; i<GDIM; i++) v[i] = h[i] * d / sin_auto(d);
  v[3] = 0;
  return v;
  }

/** more precise */
EX hyperpoint inverse_exp_newton(hyperpoint h, int iter) {
  auto approx = inverse_exp(shiftless(h));
  for(int i=0; i<iter; i++) {
    transmatrix T;
    ld eps = 1e-3;
    hyperpoint cur = direct_exp(approx);
    println(hlog, approx, " error = ", hdist(cur, h), " iteration ", i, "/", iter);
    for(int i=0; i<3; i++)
      set_column(T, i, direct_exp(approx + ctangent(i, eps)) - h);
    set_column(T, 3, C03);
    approx = approx - inverse(T) * (cur - h) * eps;
    }
  return approx;
  }

EX ld geo_dist(const hyperpoint h1, const hyperpoint h2, flagtype prec IS(pNORMAL)) {
  if(!nonisotropic) return hdist(h1, h2);
  return hypot_d(3, inverse_exp(shiftless(nisot::translate(h1, -1) * h2, prec)));
  }

EX ld geo_dist(const shiftpoint h1, const shiftpoint h2, flagtype prec IS(pNORMAL)) {
  if(!nonisotropic) return hdist(h1, h2);
  return hypot_d(3, inverse_exp(shiftless(nisot::translate(h1.h, -1) * h2.h, h2.shift - h1.shift), prec));
  }

EX ld geo_dist_q(const hyperpoint h1, const hyperpoint h2, flagtype prec IS(pNORMAL)) {
  auto d = geo_dist(h1, h2, prec);
  if(elliptic && d > 90._deg) return M_PI - d;
  return d;
  }

EX hyperpoint lp_iapply(const hyperpoint h) {
  return nisot::local_perspective_used ? inverse(NLP) * h : h;
  }

EX hyperpoint lp_apply(const hyperpoint h) {
  return nisot::local_perspective_used ? NLP * h : h;
  }

EX hyperpoint smalltangent() { return xtangent(.1); }

EX void cyclefix(ld& a, ld b) {
  while(a > b + M_PI) a -= TAU;
  while(a < b - M_PI) a += TAU;
  }

EX ld raddif(ld a, ld b) {
  ld d = a-b;
  if(d < 0) d = -d;
  if(d > TAU) d -= TAU;
  if(d > M_PI) d = TAU-d;
  return d;
  }

EX int bucket_scale = 10000;

EX unsigned bucketer(ld x) {
  return (unsigned) (long long) (floor(x * bucket_scale + .5));
  }

EX unsigned bucketer(hyperpoint h) {
  unsigned dx = 0;
  if(gproduct) {
    auto d = product_decompose(h);
    h = d.second;
    dx += bucketer(d.first) * 50;
    if(cgi.emb->is_euc_in_product() && in_h2xe()) h /= h[2];
    }
  dx += bucketer(h[0]) + 1000 * bucketer(h[1]) + 1000000 * bucketer(h[2]);
  if(MDIM == 4) dx += bucketer(h[3]) * 1000000001;
  if(elliptic) dx = min(dx, -dx);
  return dx;
  }  

#if MAXMDIM >= 4
/** @brief project the origin to the triangle [h1,h2,h3] */
EX hyperpoint project_on_triangle(hyperpoint h1, hyperpoint h2, hyperpoint h3) {
  h1 /= h1[3];
  h2 /= h2[3];
  h3 /= h3[3];
  transmatrix T;
  T[0] = h1; T[1] = h2; T[2] = h3;
  T[3] = C0;
  ld det_orig = det3(T);
  hyperpoint orthogonal = (h2 - h1) ^ (h3 - h1);
  T[0] = orthogonal; T[1] = h2-h1; T[2] = h3-h1;
  ld det_orth = det3(T);
  hyperpoint result = orthogonal * (det_orig / det_orth);
  result[3] = 1;
  return normalize(result);
  }
#endif

EX hyperpoint lerp(hyperpoint a0, hyperpoint a1, ld x) {
  return a0 + (a1-a0) * x;
  }

EX hyperpoint linecross(hyperpoint a, hyperpoint b, hyperpoint c, hyperpoint d) {  
  a /= a[LDIM];
  b /= b[LDIM];
  c /= c[LDIM];
  d /= d[LDIM];
  
  ld bax = b[0] - a[0];
  ld dcx = d[0] - c[0];
  ld cax = c[0] - a[0];
  ld bay = b[1] - a[1];
  ld dcy = d[1] - c[1];
  ld cay = c[1] - a[1];
  
  hyperpoint res;
  res[0] = (cay * dcx * bax + a[0] * bay * dcx - c[0] * dcy * bax) / (bay * dcx - dcy * bax);
  res[1] = (cax * dcy * bay + a[1] * bax * dcy - c[1] * dcx * bay) / (bax * dcy - dcx * bay);
  res[2] = 0;
  res[3] = 0;
  res[GDIM] = 1;
  return normalize(res);  
  }

EX ld inner2(hyperpoint h1, hyperpoint h2) {
  return 
    hyperbolic ? h1[LDIM] * h2[LDIM] - h1[0] * h2[0] - h1[1] * h2[1] :
    sphere ? h1[LDIM] * h2[LDIM] + h1[0] * h2[0] + h1[1] * h2[1] :
    h1[0] * h2[0] + h1[1] * h2[1];
  }

EX hyperpoint circumscribe(hyperpoint a, hyperpoint b, hyperpoint c) {
  hyperpoint h = C0;

  b = b - a;
  c = c - a;
  
  if(euclid) {
    ld b2 = inner2(b, b)/2;
    ld c2 = inner2(c, c)/2;
    
    ld det = c[1]*b[0] - b[1]*c[0];
    
    h = a;
    
    h[1] += (c2*b[0] - b2 * c[0]) / det;
    h[0] += (c2*b[1] - b2 * c[1]) / -det;
    
    return h;
    }

  if(inner2(b,b) < 0) {
    b = b / sqrt(-inner2(b, b));
    c = c + b * inner2(c, b);
    h = h + b * inner2(h, b);
    }
  else {
    b = b / sqrt(inner2(b, b));
    c = c - b * inner2(c, b);
    h = h - b * inner2(h, b);
    }
  
  if(inner2(c,c) < 0) {
    c = c / sqrt(-inner2(c, c));
    h = h + c * inner2(h, c);
    }
  else {
    c = c / sqrt(inner2(c, c));
    h = h - c * inner2(h, c);
    }
  
  if(h[LDIM] < 0) h[0] = -h[0], h[1] = -h[1], h[LDIM] = -h[LDIM];

  ld i = inner2(h, h);
  if(i > 0) h /= sqrt(i);
  else h /= -sqrt(-i);

  return h;
  }

EX ld inner3(hyperpoint h1, hyperpoint h2) {
  return 
    hyperbolic ? h1[LDIM] * h2[LDIM] - h1[0] * h2[0] - h1[1] * h2[1]  - h1[2]*h2[2]:
    sphere ? h1[LDIM] * h2[LDIM] + h1[0] * h2[0] + h1[1] * h2[1] + h1[2]*h2[2]:
    h1[0] * h2[0] + h1[1] * h2[1];
  }

#if MAXMDIM >= 4
/** circumscribe for H3 and S3 (not for E3 yet!) */
EX hyperpoint circumscribe(hyperpoint a, hyperpoint b, hyperpoint c, hyperpoint d) {
  
  array<hyperpoint, 4> ds = { b-a, c-a, d-a, C0 };
  
  for(int i=0; i<3; i++) {
  
    if(inner3(ds[i],ds[i]) < 0) {
      ds[i] = ds[i] / sqrt(-inner3(ds[i], ds[i]));
      for(int j=i+1; j<4; j++)
        ds[j] = ds[j] + ds[i] * inner3(ds[i], ds[j]);
      }
    else {
      ds[i] = ds[i] / sqrt(inner3(ds[i], ds[i]));
      for(int j=i+1; j<4; j++)
        ds[j] = ds[j] - ds[i] * inner3(ds[i], ds[j]);          
      }
    }
  
  hyperpoint& h = ds[3];
  
  if(h[3] < 0) h = -h;

  ld i = inner3(h, h);
  if(i > 0) h /= sqrt(i);
  else h /= -sqrt(-i);

  return h;
  }
#endif

/** the point in distance dist from 'material' to 'dir' (usually an (ultra)ideal point) */
EX hyperpoint towards_inf(hyperpoint material, hyperpoint dir, ld dist IS(1)) {
  transmatrix T = gpushxto0(material);
  hyperpoint id = T * dir;
  return rgpushxto0(material) * rspintox(id) * xpush0(dist);
  }

EX bool clockwise(hyperpoint h1, hyperpoint h2) {
  return h1[0] * h2[1] > h1[1] * h2[0];
  }

EX ld worst_precision_error;

#if HDR
struct hr_precision_error : hr_exception { hr_precision_error() : hr_exception("precision error") {} };
#endif

/** check if a and b are the same, testing for equality. Throw an exception or warning if not sure */
EX bool same_point_may_warn(hyperpoint a, hyperpoint b) {
  ld d = hdist(a, b);
  if(d > 1e-2) return false;
  if(d > 1e-3) throw hr_precision_error();
  if(d > 1e-6 && worst_precision_error <= 1e-6)
    addMessage("warning: precision errors are building up!");
  if(d > worst_precision_error) worst_precision_error = d;
  return true;
  }

}
