// Hyperbolic Rogue
// Copyright (C) 2011-2016 Zeno Rogue, see 'hyper.cpp' for details

// implementation of the Hypersian Rug mode


#if CAP_RUG

#define TEXTURESIZE (texturesize)
#define HTEXTURESIZE (texturesize/2)

#if !CAP_GLEW
#if ISLINUX
extern "C" {
GLAPI void APIENTRY glGenFramebuffers (GLsizei n, GLuint *framebuffers);
GLAPI void APIENTRY glBindFramebuffer (GLenum target, GLuint framebuffer);
GLAPI void APIENTRY glFramebufferTexture (GLenum target, GLenum attachment, GLuint texture, GLint level);
GLAPI GLenum APIENTRY glCheckFramebufferStatus (GLenum target);
GLAPI void APIENTRY glDrawBuffers (GLsizei n, const GLenum *bufs);
GLAPI void APIENTRY glGenRenderbuffers (GLsizei n, GLuint *renderbuffers);
GLAPI void APIENTRY glBindRenderbuffer (GLenum target, GLuint renderbuffer);
GLAPI void APIENTRY glRenderbufferStorage (GLenum target, GLenum internalformat, GLsizei width, GLsizei height);
GLAPI void APIENTRY glFramebufferRenderbuffer (GLenum target, GLenum attachment, GLenum renderbuffertarget, GLuint renderbuffer);
GLAPI void APIENTRY glDeleteRenderbuffers (GLsizei n, const GLuint *renderbuffers);
GLAPI void APIENTRY glDeleteFramebuffers (GLsizei n, const GLuint *framebuffers);
}
#endif

#if ISMAC
#define glFramebufferTexture glFramebufferTextureEXT 
#endif
#endif

namespace rug {

static const bool fast_euclidean = true;

int torus_precision = 2;

eGeometry gwhere = gEuclid;

// hypersian rug datatypes and globals
//-------------------------------------

bool rugged = false;
bool genrug = false;
bool glew   = false;

bool renderonce  = false;
bool rendernogl  = false;
int  texturesize = 1024;
ld scale = 1;

ld err_zero = 1e-3;

int queueiter, qvalid, dt;
double err;

struct edge {
  struct rugpoint *target;
  double len;
  };

struct rugpoint {
  double x1, y1;
  bool valid;
  bool inqueue;
  double dist;
  hyperpoint h;
  hyperpoint flat;
  vector<edge> edges;
  };

struct triangle {
  rugpoint *m[3];
  triangle(rugpoint *m1, rugpoint *m2, rugpoint *m3) {
    m[0] = m1; m[1] = m2; m[2] = m3;
    }
  };

vector<rugpoint*> points;
vector<triangle> triangles;

bool rug_perspective = false;

// extra geometry functions
//--------------------------

// returns a matrix M
// such that inverse(M) * h1 = ( |h1|, 0, 0) and inverse(M) * h2 = ( .., .., 0)

bool zero2(hyperpoint h) { using namespace hyperpoint_vec; return !(h|h); }

transmatrix orthonormalize(hyperpoint h1, hyperpoint h2) {
  using namespace hyperpoint_vec;

  hyperpoint vec[3];
  vec[2] = (h1 ^ h2);
  if(zero2(vec[2])) vec[2] = hpxyz(.519, .4619, .136);

  vec[2] = vec[2] / sqrt(vec[2]|vec[2]);
  vec[0] = h1 - vec[2] * (h1|vec[2]);
  if(zero2(vec[0])) {
    vec[0] = hpxyz(1.641,2,3); 
    vec[0] = vec[0] - vec[2] * (vec[0]|vec[2]);
    }
  vec[0] = vec[0] / sqrt(vec[0]|vec[0]);
  vec[1] = h2 - vec[2] * (h2|vec[2]) - vec[0] * (h2|vec[0]);
  if(zero2(vec[1])) {
    vec[1] = hpxyz(1.713,3,5); 
    vec[1] = vec[1] - vec[2] * (vec[1]|vec[2]) - vec[0] * (vec[1]|vec[0]);
    }
  vec[1] = vec[1] / sqrt(vec[1]|vec[1]);
  
  transmatrix M;
  for(int i=0; i<3; i++) for(int j=0; j<3; j++)
    M[i][j] = vec[j][i];
  
  return M;
  }

hyperpoint azeq_to_hyperboloid(hyperpoint h) {
  if(abs(h[2])>1e-4) printf("Error: h[2] = %lf\n", h[2]);
  if(euclid) {
    h[2] = 1;
    return h;
    }
  ld d = hypot(h[0], h[1]);
  if(d == 0) {
    h[2] = 1;
    return h;
    }
  if(sphere) {
    h[0] = sin(d) * h[0]/d;
    h[1] = sin(d) * h[1]/d;
    h[2] = cos(d);
    }
  else {
    h[0] = sinh(d) * h[0]/d;
    h[1] = sinh(d) * h[1]/d;
    h[2] = cosh(d);
    }
  return h;
  }

hyperpoint hyperboloid_to_azeq(hyperpoint h) {
  if(euclid) {
    h[2] = 0;
    return h;
    }
  else {
    ld d = hdist0(h);
    if(d == 0) { h[2] = 0; return h; }
    ld d2 = hypot(h[0], h[1]);
    if(d2 == 0) { h[2] = 0; return h; }
    h[0] = d * h[0] / d2;
    h[1] = d * h[1] / d2;
    h[2] = 0;
    return h;
    }
  }

void push_all_points(int coord, ld val) {
  if(!val) return;
  else if(fast_euclidean && gwhere == gEuclid) {
    for(int i=0; i<size(points); i++) 
      points[i]->flat[coord] += val;
    }
  else for(int i=0; i<size(points); i++) {
    dynamicval<eGeometry> gw(geometry, gwhere);
    transmatrix M = orthonormalize(hpxyz(coord==0,coord==1,coord==2), points[i]->flat);
    transmatrix Mi = inverse(M);
    hyperpoint f = azeq_to_hyperboloid(Mi * points[i]->flat);
    points[i]->flat = M * hyperboloid_to_azeq(xpush(val) * f);
    }
  }

// construct the graph
//---------------------

int hyprand;

rugpoint *addRugpoint(hyperpoint h, double dist) {
  rugpoint *m = new rugpoint;
  m->h = h;
  
  /*
  ld tz = vid.alphax+h[2];
  m->x1 = (1 + h[0] / tz) / 2;
  m->y1 = (1 + h[1] / tz) / 2;
  */

  hyperpoint onscreen;
  applymodel(m->h, onscreen);
  m->x1 = (1 + onscreen[0] * vid.scale) / 2;
  m->y1 = (1 + onscreen[1] * vid.scale) / 2;

  m->flat = // hpxyz(h[0], h[1], sin(atan2(h[0], h[1]) * 3 + hyprand) * (h[2]-1) / 1000);
    hpxyz(h[0], h[1], (h[2]-1) * (rand() % 1000 - rand() % 1000) / 1000);
  
  if(rug_perspective && gwhere == gEuclid) m->flat[2] -= 3;
  m->valid = false;
  m->inqueue = false;
  m->dist = dist;
  points.push_back(m);
  return m;
  }

rugpoint *findRugpoint(hyperpoint h, double dist) {
  for(int i=0; i<size(points); i++) 
    if(intval(points[i]->h, h) < 1e-5) return points[i];
  return addRugpoint(h, dist);
  }

void addNewEdge(rugpoint *e1, rugpoint *e2) {
  edge e; e.target = e2; e1->edges.push_back(e);
  e.target = e1; e2->edges.push_back(e);
  }

void addEdge(rugpoint *e1, rugpoint *e2) {
  for(int i=0; i<size(e1->edges); i++) 
    if(e1->edges[i].target == e2) return;
  addNewEdge(e1, e2);
  }

void addTriangle(rugpoint *t1, rugpoint *t2, rugpoint *t3) {
  addEdge(t1, t2); addEdge(t2, t3); addEdge(t3, t1);
  triangles.push_back(triangle(t1,t2,t3));
  }

void addTriangle1(rugpoint *t1, rugpoint *t2, rugpoint *t3) {
  rugpoint *t12 = findRugpoint(mid(t1->h, t2->h), (t1->dist+ t2->dist)/2);
  rugpoint *t23 = findRugpoint(mid(t2->h, t3->h), (t1->dist+ t2->dist)/2);
  rugpoint *t31 = findRugpoint(mid(t3->h, t1->h), (t1->dist+ t2->dist)/2);
  addTriangle(t1,  t12, t31);
  addTriangle(t12, t2,  t23);
  addTriangle(t23, t3,  t31);
  addTriangle(t23, t31, t12);
  }

bool psort(rugpoint *a, rugpoint *b) {
  return hdist0(a->h) < hdist0(b->h);
  }

ld modelscale = 1;

void calcLengths() {
  for(int i=0; i<size(points); i++) for(int j=0; j<size(points[i]->edges); j++) {
    ld d = hdist(points[i]->h, points[i]->edges[j].target->h);
    if(elliptic && d > M_PI/2) d = M_PI - d;
    points[i]->edges[j].len = d * modelscale;
    }
  }

void setVidParam() {
  vid.xres = vid.yres = TEXTURESIZE;
  vid.scrsize = HTEXTURESIZE;
  vid.radius = vid.scrsize * vid.scale; vid.xcenter = HTEXTURESIZE; vid.ycenter = HTEXTURESIZE;
  vid.beta = 2; vid.alphax = 1; vid.eye = 0; vid.goteyes = false;
  }

void buildTorusRug() {
  using namespace torusconfig;

  setVidParam();

  struct toruspoint {
    int x,y;
    toruspoint() { x=qty; y=qty; }
    toruspoint(int _x, int _y) : x(_x), y(_y) {}
    int d2() { 
      return x*x+(euclid6?x*y:0)+y*y;
      }
    };
  
  vector<toruspoint> zeropoints;
  vector<toruspoint> tps(qty);
  
  for(int ax=-qty; ax<qty; ax++)
  for(int ay=-qty; ay<qty; ay++) {
    int v = (ax*dx + ay*dy) % qty;
    if(v<0) v += qty;
    toruspoint tp(ax, ay);
    if(tps[v].d2() > tp.d2()) tps[v] = tp;
    if(v == 0) 
      zeropoints.emplace_back(ax, ay);
    }
  
  pair<toruspoint, toruspoint> solution;
  ld bestsol = 1e12;
    
  for(auto p1: zeropoints)
  for(auto p2: zeropoints) {
    int det = p1.x * p2.y - p2.x * p1.y;
    if(det < 0) continue;
    if(det != qty && det != -qty) continue;
    ld quality = ld(p1.d2()) * p2.d2();
    if(quality < bestsol * 3)
    if(quality < bestsol)
      bestsol = quality, solution.first = p1, solution.second = p2;
    }
  
  if(solution.first.d2() > solution.second.d2())
    swap(solution.first, solution.second);
    
  ld factor = sqrt(ld(solution.second.d2()) / solution.first.d2());
  
  printf("factor = %lf\n", factor);
  if(factor < 2) factor = 2.2;
  factor -= 1;
        
  // 22,1
  // 7,-17
  
  // transmatrix z1 = {{{22,7,0}, {1,-17,0}, {0,0,1}}};
  transmatrix z1 = {{{(ld)solution.first.x,(ld)solution.second.x,0}, {(ld)solution.first.y,(ld)solution.second.y,0}, {0,0,1}}};
  transmatrix z2 = inverse(z1);
  
  auto addToruspoint = [&] (ld x, ld y) {
    auto r = addRugpoint(C0, 0);
    hyperpoint onscreen;
    applymodel(tC0(eumove(x, y)), onscreen);
    // take point (1,0)
    // apply eumove(1,0)
    // divide by EUCSCALE
    // multiply by vid.radius (= HTEXTURESIZE * rugzoom)
    // add 1, divide by texturesize
    r->x1 = onscreen[0];
    r->y1 = onscreen[1];
    // r->y1 = (1 + onscreen[1] * rugzoom / EUCSCALE)/2;
    hyperpoint h1 = hpxyz(x, y, 0);
    hyperpoint h2 = z2 * h1;
    double alpha = -h2[0] * 2 * M_PI;
    double beta = -h2[1] * 2 * M_PI;
    // r->flat = {alpha, beta, 0}; 
    double sc = (factor+1)/4;
    r->flat = hpxyz((factor+cos(alpha)) * cos(beta) * sc, (factor+cos(alpha)) * sin(beta) * sc, -sin(alpha) * sc);
    r->valid = true;
    return r;
    };
  
  int rugmax = min(torus_precision, 16);
  ld rmd = rugmax;
    
  for(int i=0; i<qty; i++) {
    int x = tps[i].x, y = tps[i].y;
    rugpoint *rugarr[32][32];
    for(int yy=0; yy<=rugmax; yy++)
    for(int xx=0; xx<=rugmax; xx++)
      rugarr[yy][xx] = addToruspoint(x+(xx-yy)/rmd, y+yy/rmd);
    
    for(int yy=0; yy<rugmax; yy++)
    for(int xx=0; xx<rugmax; xx++)
      addTriangle(rugarr[yy][xx], rugarr[yy+1][xx], rugarr[yy+1][xx+1]),
      addTriangle(rugarr[yy][xx], rugarr[yy][xx+1], rugarr[yy+1][xx+1]);
    }
  
  double maxz = 0;
  
  for(auto p: points)
    maxz = max(maxz, max(abs(p->x1), abs(p->y1)));
  
  // maxz * rugzoom * vid.radius == vid.radius
  
  vid.scale = 1 / maxz;
  
  for(auto p: points)
    p->x1 = (vid.xcenter + vid.radius * vid.scale * p->x1)/ vid.xres,
    p->y1 = (vid.ycenter - vid.radius * vid.scale * p->y1)/ vid.yres;
  
  return;
  }

void buildRug() {

  if(torus) {
    buildTorusRug();
    return;
    }
  
  map<cell*, rugpoint *> vptr;
 
  for(int i=0; i<size(dcal); i++)
    if(gmatrix.count(dcal[i])) 
      vptr[dcal[i]] = addRugpoint(gmatrix[dcal[i]]*C0, dcal[i]->cpdist);

  for(int i=0; i<size(dcal); i++) {
    cell *c = dcal[i];
    rugpoint *v = vptr[c];
    if(!v) continue;
    for(int j=0; j<c->type; j++) {
      cell *c2 = c->mov[j];
      rugpoint *w = vptr[c2];
      if(!w) continue;
      // if(v<w) addEdge(v, w);
      
      cell *c3 = c->mov[(j+1) % c->type];
      rugpoint *w2 = vptr[c3];
      if(!w2) continue;
      if(ctof(c)) addTriangle(v, w, w2);
      }
    }

  printf("vertices = %d triangles=  %d\n", size(points), size(triangles));

  calcLengths();
  sort(points.begin(), points.end(), psort);
  }

// rug physics

queue<rugpoint*> pqueue;
void enqueue(rugpoint *m) {
  if(m->inqueue) return;
  pqueue.push(m);
  m->inqueue = true;
  }

void force_euclidean(rugpoint& m1, rugpoint& m2, double rd, double d1=1, double d2=1) {
  if(!m1.valid || !m2.valid) return;
  // double rd = hdist(m1.h, m2.h) * xd;
  // if(rd > rdz +1e-6 || rd< rdz-1e-6) printf("%lf %lf\n", rd, rdz);
  double t = 0;
  for(int i=0; i<3; i++) t += (m1.flat[i] - m2.flat[i]) * (m1.flat[i] - m2.flat[i]);
  t = sqrt(t);
  // printf("%lf %lf\n", t, rd);
  err += (t-rd) * (t-rd);
  bool nonzero = abs(t-rd) > err_zero;
  double force = (t - rd) / t / 2; // 20.0;
  for(int i=0; i<3; i++) {
    double di = (m2.flat[i] - m1.flat[i]) * force;
    m1.flat[i] += di * d1;
    m2.flat[i] -= di * d2;
    if(nonzero && d2>0) enqueue(&m2);
    }  
  }

void force(rugpoint& m1, rugpoint& m2, double rd, double d1=1, double d2=1) {
  if(!m1.valid || !m2.valid) return;
  if(gwhere == gEuclid && fast_euclidean) {
    force_euclidean(m1, m2, rd, d1, d2);
    return;
    }
  // double rd = hdist(m1.h, m2.h) * xd;
  // if(rd > rdz +1e-6 || rd< rdz-1e-6) printf("%lf %lf\n", rd, rdz);
  using namespace hyperpoint_vec;
  dynamicval<eGeometry> gw(geometry, gwhere);
  transmatrix M = orthonormalize(m1.flat, m2.flat);
  transmatrix Mi = inverse(M);
  hyperpoint f1 = azeq_to_hyperboloid(Mi * m1.flat);
  hyperpoint f2 = azeq_to_hyperboloid(Mi * m2.flat);
  
  ld t = hdist(f1, f2);
  err += (t-rd) * (t-rd);
  bool nonzero = abs(t-rd) > err_zero;
  double forcev = (t - rd) / 2; // 20.0;
  
  transmatrix T = gpushxto0(f1);
  transmatrix T1 = spintox(T * f2) * T;

  transmatrix iT1 = inverse(T1);
  
  for(int i=0; i<3; i++) if(isnan(m1.flat[i])) { printf("NAN!\n"); exit(1); }

/*  
  printf("%p %p\n", &m1, &m2);

  printf("m1      = %s\n", display(m1.flat));
  printf("m2      = %s\n", display(m2.flat));
  printf("Mi * m1 = %s\n", display(Mi*m1.flat));
  printf("Mi * m2 = %s\n", display(Mi*m2.flat));

  printf("     f1 = %s\n", display(f1));
  printf(" T * f1 = %s\n", display(T * f1));
  printf("T1 * f1 = %s\n", display(T1 * f1));
  printf("     f2 = %s\n", display(f2));
  printf(" T * f2 = %s\n", display(T * f2));
  printf("T1 * f2 = %s\n", display(T1 * f2));
  printf("iT1     = %s\n", display(iT1 * C0));
  printf("iT1 + t = %s\n", display(iT1 * xpush(t) * C0));
*/

  f1 = iT1 * xpush(forcev) * C0;
  f2 = iT1 * xpush(t-forcev) * C0;

  m1.flat = M * hyperboloid_to_azeq(f1);
  m2.flat = M * hyperboloid_to_azeq(f2);
  
  if(nonzero && d2>0) enqueue(&m2);
  }

void preset(rugpoint *m) {
  int q = 0;
  hyperpoint h;
  for(int i=0; i<3; i++) h[i] = 0;
  using namespace hyperpoint_vec;
  
  for(int j=0; j<size(m->edges); j++)
  for(int k=0; k<j; k++) {
    rugpoint *a = m->edges[j].target;
    rugpoint *b = m->edges[k].target;
    if(!a->valid) continue;
    if(!b->valid) continue;
    double blen = -1;
    for(int j2=0; j2<size(a->edges); j2++) 
      if(a->edges[j2].target == b) blen = a->edges[j2].len;
    if(blen <= 0) continue;
    for(int j2=0; j2<size(a->edges); j2++) 
    for(int k2=0; k2<size(b->edges); k2++) 
    if(a->edges[j2].target == b->edges[k2].target && a->edges[j2].target != m) {    
      rugpoint *c = a->edges[j2].target;
      if(!c->valid) continue;

      double a1 = m->edges[j].len/blen;
      double a2 = m->edges[k].len/blen;
      double c1 = a->edges[j2].len/blen;
      double c2 = b->edges[k2].len/blen;
      
      double cz = (c1*c1-c2*c2+1) / 2;
      double ch = sqrt(c2*c2 - cz*cz);

      double az = (a1*a1-a2*a2+1) / 2;
      double ah = sqrt(a2*a2 - az*az);
      
      // c->h = a->h + (b->h-a->h) * cz + ch * ort
      hyperpoint ort = (c->flat - a->flat - cz * (b->flat-a->flat)) / ch;

      // m->h = a->h + (b->h-a->h) * az - ah * ort
      hyperpoint res = a->flat + (b->flat-a->flat) * az - ah * ort;
      
      for(int i=0; i<3; i++) h[i] += res[i];
      
      q++;
      }
    }
    
  if(q>0) for(int i=0; i<3; i++) m->flat[i] = h[i]/q;
  }

int divides = 0;
bool stop = false;

void subdivide() {
  int N = size(points);
  // if(euclid && gwhere == gEuclid) return;
  if(N >= 8000) {stop = true; return; }
  printf("subdivide (%d,%d)\n", N, size(triangles));
  divides++;
  vector<triangle> otriangles = triangles;
  triangles.clear();
  
  // subdivide edges
  for(int i=0; i<N; i++) {
    rugpoint *m = points[i];
    for(int j=0; j<size(m->edges); j++) {
      rugpoint *m2 = m->edges[j].target;
      rugpoint *mm = addRugpoint(mid(m->h, m2->h), (m->dist+m2->dist)/2);
      using namespace hyperpoint_vec;
      mm->flat = (m->flat + m2->flat) / 2;
      mm->valid = true; qvalid++;
      mm->inqueue = false; enqueue(mm);
      }
     m->edges.clear();
     }
    
  for(int i=0; i<size(otriangles); i++)
    addTriangle1(otriangles[i].m[0], otriangles[i].m[1], otriangles[i].m[2]);
    
  calcLengths();

  printf("result (%d,%d)\n", size(points), size(triangles));
  }

void addNewPoints() {

  if(qvalid == size(points)) {
    subdivide();
    return;
    }
  
  double dist = hdist0(points[qvalid]->h) + .1e-6;
  
  int oqvalid = qvalid;

  for(int i=0; i<size(points); i++) {
    rugpoint& m = *points[i];
    bool wasvalid = m.valid;
    m.valid = wasvalid || sphere || hdist0(m.h) <= dist;
    if(m.valid && !wasvalid) {
      qvalid++;
      if(i > 7) preset(&m);
      
      for(int it=0; it<50; it++) 
      for(int j=0; j<size(m.edges); j++)
        force(m, *m.edges[j].target, m.edges[j].len, 1, 0);

      enqueue(&m);
      }
    }
  if(qvalid != oqvalid) { printf("%4d %4d %4d %.9lf %9d %9d\n", oqvalid, qvalid, size(points), dist, dt, queueiter); }
  }

void physics() {

  if(torus) return;
  
  int t = SDL_GetTicks();
  
  err = 0;
  
  while(SDL_GetTicks() < t + 5 && !stop)
  for(int it=0; it<50; it++)
    if(pqueue.empty()) addNewPoints();
    else {
      queueiter++;
      rugpoint *m = pqueue.front();
      pqueue.pop();
      m->inqueue = false;
      for(int j=0; j<size(m->edges); j++)
        force(*m, *m->edges[j].target, m->edges[j].len);      
      }
    
  if(!stop) printf("%5d %10.7lf D%d Q%3d Qv%5d\n", queueiter, err, divides, size(pqueue), qvalid);
  }

// drawing the Rug
//-----------------

int eyemod;

void getco(rugpoint& m, double& x, double& y, double& z, int &spherepoints) {
  x = m.flat[0];
  y = m.flat[1];
  z = m.flat[2];
  if(gwhere == gSphere && z > 0) {
    ld rad = sqrt(x*x + y*y + z*z);
    // turn M_PI to -M_PI
    ld rad_to = M_PI + M_PI - rad;
    ld r = -rad_to / rad;
    x *= r;
    y *= r;
    z *= r;
    spherepoints++;
    }
  if(eyemod) x += eyemod * z * vid.eye;
  }

extern int besti;

void drawTriangle(triangle& t) {
  rugpoint& m1 = *t.m[0];
  rugpoint& m2 = *t.m[1];
  rugpoint& m3 = *t.m[2];
  if(!m1.valid || !m2.valid || !m3.valid) return;
  if(m1.dist >= sightrange+.51 || m2.dist >= sightrange+.51 || m3.dist >= sightrange+.51)
    return;
  dt++;
  int spherepoints = 0;
  double x1, y1, z1;
  double x2, y2, z2;
  double x3, y3, z3;
  getco(m1,x1,y1,z1, spherepoints);
  getco(m2,x2,y2,z2, spherepoints);
  getco(m3,x3,y3,z3, spherepoints);
  if(spherepoints == 1 || spherepoints == 2) return;
  
  double xa = x2-x1, ya = y2-y1, za = z2-z1;
  double xb = x3-x1, yb = y3-y1, zb = z3-z1;
  
  double xn = ya * zb - za * yb;
  double yn = za * xb - xa * zb;
  double zn = xa * yb - ya * xb;
  double h = sqrt(xn*xn+yn*yn+zn*zn);
  
  glNormal3f(xn/h,yn/h,zn/h);
  
  glTexCoord2f(m1.x1, m1.y1); 
  glVertex3f(x1, y1, z1);
  glTexCoord2f(m2.x1, m2.y1); 
  glVertex3f(x2, y2, z2);
  glTexCoord2f(m3.x1, m3.y1); 
  glVertex3f(x3, y3, z3);
  }

GLuint FramebufferName = 0;
GLuint renderedTexture = 0;
GLuint depth_stencil_rb = 0;

SDL_Surface *texture;
Uint32 *expanded_data;

void initTexture() {

  if(!rendernogl) {
#if !ISPANDORA
    FramebufferName = 0;
    glGenFramebuffers(1, &FramebufferName);
    glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
    
    glGenTextures(1, &renderedTexture);
    glBindTexture(GL_TEXTURE_2D, renderedTexture);
    glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, TEXTURESIZE, TEXTURESIZE, 0,GL_RGB, GL_UNSIGNED_BYTE, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  
#ifdef TEX
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, renderedTexture, 0);  
#else
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderedTexture, 0);  
#endif
    GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, DrawBuffers);
    
    glGenRenderbuffers(1, &depth_stencil_rb);
    glBindRenderbuffer(GL_RENDERBUFFER, depth_stencil_rb);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, TEXTURESIZE, TEXTURESIZE);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_stencil_rb);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depth_stencil_rb);
    
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      addMessage("Failed to initialize the framebuffer");
      rugged = false;
      }  
#endif
    }
  else {
    texture = SDL_CreateRGBSurface(SDL_SWSURFACE,TEXTURESIZE,TEXTURESIZE,32,0,0,0,0);  
    glGenTextures( 1, &renderedTexture );
    glBindTexture( GL_TEXTURE_2D, renderedTexture);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    expanded_data = new Uint32[TEXTURESIZE * TEXTURESIZE];
    }
  }

void prepareTexture() {
  videopar svid = vid;
  
  setVidParam();
  
  if(rendernogl) {
    vid.usingGL = false;
    SDL_Surface *sav = s;
    s = texture;
    SDL_FillRect(s, NULL, 0);

    drawfullmap();
    s = sav;
    for(int y=0; y<TEXTURESIZE; y++) for(int x=0; x<TEXTURESIZE; x++)
      expanded_data[y*TEXTURESIZE + x] = qpixel(texture, x, TEXTURESIZE-1-y) | 0xFF000000;
    glBindTexture( GL_TEXTURE_2D, renderedTexture);
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, TEXTURESIZE, TEXTURESIZE, 0, GL_BGRA, GL_UNSIGNED_BYTE, expanded_data );    
    }
  else { 
#if !ISPANDORA
    glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
    glViewport(0,0,TEXTURESIZE,TEXTURESIZE);
  
    setGLProjection();
    ptds.clear();
    drawthemap();
    if(!renderonce) {
      for(int i=0; i<numplayers(); i++) if(multi::playerActive(i))
        queueline(tC0(shmup::ggmatrix(playerpos(i))), mouseh, 0xFF00FF, 8);
      }
    drawqueue();  
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
#endif
    }
  vid = svid;
  if(!rendernogl) glViewport(0,0,vid.xres,vid.yres);
  }

void closeTexture() {
  if(rendernogl) {
    SDL_FreeSurface(texture);
    glDeleteTextures(1, &renderedTexture);
    delete[] expanded_data;
    }
  else {
#if !ISPANDORA
    glDeleteTextures(1, &renderedTexture);
    glDeleteRenderbuffers(1, &depth_stencil_rb);
    glDeleteFramebuffers(1, &FramebufferName);
#endif
    }
  }

double xview, yview;

void glcolorClear(int color) {
  unsigned char *c = (unsigned char*) (&color);
  glClearColor(c[3] / 255.0, c[2] / 255.0, c[1]/255.0, c[0] / 255.0);
  }

void drawRugScene() {
  GLfloat light_ambient[] = { 3.5, 3.5, 3.5, 1.0 };
  GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat light_position[] = { 0.0, 0.0, 0.0, 1.0 };

  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);

  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
  GLERR("lighting");

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  glBindTexture(GL_TEXTURE_2D, renderedTexture);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  if(backcolor == 0) 
    glClearColor(0.05,0.05,0.05,1);
  else
    glcolorClear(backcolor << 8 | 0xFF);
  glClearDepth(1.0f); 
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  glDisable(GL_BLEND);
  glEnable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  
  if(rug_perspective) {
    ld vnear = .05;
    ld vfar = 10;
    ld sca = vnear / 2 / vid.xres;
    glFrustum(-sca * vid.xres, sca * vid.xres, -sca * vid.yres, sca * vid.yres, vnear, vfar);
    }
  else {
    xview = vid.xres/(vid.scrsize*scale);
    yview = vid.yres/(vid.scrsize*scale);

    glOrtho(-xview, xview, -yview, yview, -1000, 1000);
    }

  glColor4f(1,1,1,1);
  
  if(vid.eye > .001 || vid.eye < -.001) { 
    selectEyeMask(1);
    glClear(GL_DEPTH_BUFFER_BIT);
    glBegin(GL_TRIANGLES);
    eyemod = 1;
    for(int t=0; t<size(triangles); t++)
      drawTriangle(triangles[t]);
    glEnd();
    selectEyeMask(-1);
    eyemod = -1;
    glClear(GL_DEPTH_BUFFER_BIT);
    glBegin(GL_TRIANGLES);
    for(int t=0; t<size(triangles); t++)
      drawTriangle(triangles[t]);
    glEnd();
    selectEyeMask(0);
    }
  else {
    glBegin(GL_TRIANGLES);
    for(int t=0; t<size(triangles); t++)
      drawTriangle(triangles[t]);
    glEnd();
    }

  glDisable(GL_TEXTURE_2D);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  selectEyeGL(0);
  }

// organization
//--------------

transmatrix rotmatrix(double rotation, int c0, int c1) {
  transmatrix t = Id;
  t[c0][c0] = cos(rotation);
  t[c1][c1] = cos(rotation);
  t[c0][c1] = sin(rotation);
  t[c1][c0] = -sin(rotation);
  return t;
  }

transmatrix currentrot;
    
void init() {
#if CAP_GLEW
  if(!glew) { 
    glew = true; 
    GLenum err = glewInit();
    if (GLEW_OK != err) {
      addMessage("Failed to initialize GLEW");
      return;
      }
    }
#endif
  if(rugged) return;
  rugged = true;
  if(scale < .01 || scale > 100) scale = 1;
  initTexture();
  if(renderonce) prepareTexture();
  if(!rugged) return;
  
  genrug = true;
  drawthemap();
  genrug = false;
  
  buildRug();
  qvalid = 0; dt = 0; queueiter = 0;
  
  currentrot = Id;
  }

void close() {
  if(!rugged) return;
  rugged = false;
  closeTexture();
  triangles.clear();
  for(int i=0; i<size(points); i++) delete points[i];
  points.clear();
  pqueue = queue<rugpoint*> ();
  }

int lastticks;

void actDraw() { 
  if(!renderonce) prepareTexture();
  physics();
  drawRugScene();
  Uint8 *keystate = SDL_GetKeyState(NULL);
  int qm = 0;
  double alpha = (ticks - lastticks) / 1000.0;
  lastticks = ticks;

  transmatrix t = Id;

  if(rug_perspective) {
    int qms = 0;
    if(keystate[SDLK_HOME]) qm++, t = t * rotmatrix(alpha, 0, 1);
    if(keystate[SDLK_END]) qm++, t = t * rotmatrix(alpha, 1, 0);
    
    if(!keystate[SDLK_LSHIFT]) {
      if(keystate[SDLK_DOWN]) qm++, t = t * rotmatrix(alpha, 2, 1);
      if(keystate[SDLK_UP]) qm++, t =  t * rotmatrix(alpha, 1, 2);
      if(keystate[SDLK_LEFT]) qm++, t = t * rotmatrix(alpha, 2, 0);
      if(keystate[SDLK_RIGHT]) qm++, t =  t * rotmatrix(alpha, 0, 2);
      }
    ld push = 0;
    if(keystate[SDLK_PAGEDOWN]) push -= alpha;
    if(keystate[SDLK_PAGEUP]) push += alpha;
    
    ld strafex = 0, strafey = 0;
    if(keystate[SDLK_LSHIFT]) {    
      if(keystate[SDLK_LEFT]) strafex -= alpha;
      if(keystate[SDLK_RIGHT]) strafex += alpha;
      if(keystate[SDLK_UP]) strafey -= alpha;
      if(keystate[SDLK_DOWN]) strafey += alpha;
      }

    if(qm) 
    for(int i=0; i<size(points); i++) {
      points[i]->flat = t * points[i]->flat;
      }
    
    push_all_points(2, push);
    push_all_points(0, strafex);
    push_all_points(1, strafey);
    }
  else {
    if(keystate[SDLK_HOME]) qm++, t = inverse(currentrot);
    if(keystate[SDLK_END]) qm++, t = currentrot * rotmatrix(alpha, 0, 1) * inverse(currentrot);
    if(keystate[SDLK_DOWN]) qm++, t = t * rotmatrix(alpha, 1, 2);
    if(keystate[SDLK_UP]) qm++, t =  t * rotmatrix(alpha, 2, 1);
    if(keystate[SDLK_LEFT]) qm++, t = t * rotmatrix(alpha, 0, 2);
    if(keystate[SDLK_RIGHT]) qm++, t =  t * rotmatrix(alpha, 2, 0);
    if(keystate[SDLK_PAGEUP]) scale *= exp(alpha);
    if(keystate[SDLK_PAGEDOWN]) scale /= exp(alpha);
  
    if(qm) {
      currentrot = t * currentrot;
      for(int i=0; i<size(points); i++) points[i]->flat = t * points[i]->flat;
      }
    }
  }

int besti;

hyperpoint gethyper(ld x, ld y) {
  double mx = ((x*2 / vid.xres)-1) * xview;
  double my = (1-(y*2 / vid.yres)) * yview;
  double bdist = 1e12;
  
  double rx1=0, ry1=0;
  
  bool found = false;
  
  for(int i=0; i<size(triangles); i++) {
    auto r0 = triangles[i].m[0];
    auto r1 = triangles[i].m[1];
    auto r2 = triangles[i].m[2];
    double dx1 = r1->flat[0] - r0->flat[0];
    double dy1 = r1->flat[1] - r0->flat[1];
    double dx2 = r2->flat[0] - r0->flat[0];
    double dy2 = r2->flat[1] - r0->flat[1];
    double dxm = mx - r0->flat[0];
    double dym = my - r0->flat[1];
    // A (dx1,dy1) = (1,0)
    // B (dx2,dy2) = (0,1)
    double det = dx1*dy2 - dy1*dx2;
    double tx = dxm * dy2 - dym * dx2;
    double ty = -(dxm * dy1 - dym * dx1);
    tx /= det; ty /= det;
    if(tx >= 0 && ty >= 0 && tx+ty <= 1) {
      double rz1 = r0->flat[2] * (1-tx-ty) + r1->flat[2] * tx + r2->flat[2] * ty;
      rz1 = -rz1;
      if(rz1 < bdist) {
        bdist = rz1;
        rx1 = r0->x1 + (r1->x1 - r0->x1) * tx + (r2->x1 - r0->x1) * ty;
        ry1 = r0->y1 + (r1->y1 - r0->y1) * tx + (r2->y1 - r0->y1) * ty;
        }
      found = true;
      }
    }
  
  if(!found) return Hypc;
  
  double px = rx1 * TEXTURESIZE, py = (1-ry1) * TEXTURESIZE;

  videopar svid = vid;
  setVidParam();
  hyperpoint h = ::gethyper(px, py);
  vid = svid;

  return h;
  }

void show() {
  dialog::init(XLAT("hypersian rug mode"), iinf[itPalace].color, 150, 100);
  
  if((euclid || sphere) && !torus) {
    dialog::addInfo("This makes sense only in hyperbolic or Torus geometry.");
    dialog::addBreak(50);
    }

  dialog::addItem(XLAT("what's this?"), 'h');
  dialog::addItem(XLAT("take me back"), 'q');

  dialog::addItem(XLAT("enable the Hypersian Rug mode"), 'u');
    
  dialog::addBoolItem(XLAT("render the texture only once"), (renderonce), 'o');
  dialog::addBoolItem(XLAT("render texture without OpenGL"), (rendernogl), 'g');  
  dialog::addSelItem(XLAT("texture size"), its(texturesize)+"x"+its(texturesize), 's');
  if(torus) {
    if(torus_precision < 1) torus_precision = 1;
    if(torus_precision > 16) torus_precision = 16;
    dialog::addSelItem(XLAT("precision"), its(torus_precision), 'p');
    }
  dialog::display();
  keyhandler = [] (int sym, int uni) {
  #if ISPANDORA
    rendernogl = true;
  #endif
    dialog::handleNavigation(sym, uni);

    if(uni == 'h') gotoHelp(
      "In this mode, HyperRogue is played on a 3D model of a part of the hyperbolic plane, "
      "similar to one you get from the 'paper model creator' or by hyperbolic crocheting.\n\n"
      "This requires some OpenGL extensions and may crash or not work correctly -- enabling "
      "the 'render texture without OpenGL' options may be helpful in this case. Also the 'render once' option "
      "will make the rendering faster, but the surface will be rendered only once, so "
      "you won't be able to play a game on it.\n\n"
      "Use arrow keys to rotate, Page Up/Down to zoom."
      );
    else if(uni == 'u') {
      if((euclid || sphere) && !torus)
        addMessage("This makes sense only in hyperbolic or Torus geometry.");
      {        
        rug::init();
        popScreen();
        }
      }
    else if(uni == 'o')
      renderonce = !renderonce;
    else if(uni == 'p' && torus_precision)
      dialog::editNumber(torus_precision, 0, 16, 1, 2, "precision", "precision");
  #if !ISPANDORA
    else if(uni == 'g')
      rendernogl = !rendernogl;
  #endif
    else if(uni == 's') {
      texturesize *= 2;
      if(texturesize == 8192) texturesize = 128;
      dialog::scaleLog();
      }
    else if(doexiton(sym, uni)) popScreen();
    };
  }

void select() {
  if(rug::rugged) rug::close();
  else pushScreen(rug::show);
  }

int rugArgs() {
  using namespace arg;
           
  if(0) ;
  else if(argis("-rugmodelscale")) {
    shift(); modelscale = argf();
    }

  else if(argis("-ruggeo")) {
    shift(); gwhere = (eGeometry) argi();
    }

  else if(argis("-rugpers")) {
    rug_perspective = true;
    }

  else if(argis("-rugorth")) {
    rug_perspective = false;
    }

  else if(argis("-rugerr")) {
    err_zero = argf();
    }

  else return 1;
  return 0;
  }

auto rug_hook = 
  addHook(hooks_args, 100, rugArgs);
  
}

#else

// fake for mobile
namespace rug {
    bool rugged = false;
    bool renderonce = false;
    bool rendernogl = true;
    int texturesize = 512;
    ld scale = 1.0f;
}

#endif
