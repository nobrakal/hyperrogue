// Hyperbolic Rogue -- binary tilings
// Copyright (C) 2011-2019 Zeno Rogue, see 'hyper.cpp' for details

/** \file binary-tiling.cpp 
 *  \brief Binary tilings in 2D and 3D
 */

#include "hyper.h"
namespace hr {

EX namespace bt {

  /** note: nihsolv and kd3 tilings return bt::in(). They are defined elsewhere, although some of bt:: functions are used for them */
  EX bool in() {
#if CAP_BT
    return cgflags & qBINARY;
#else
    return false;
#endif
    }

#if !CAP_BT
  EX int updir() { return 0; }
#endif
  
#if CAP_BT
  #if HDR
  enum bindir {
    bd_right = 0,
    bd_up_right = 1,
    bd_up = 2,
    bd_up_left = 3,
    bd_left = 4,
    bd_down = 5, /* for cells of degree 6 */
    bd_down_left = 5, /* for cells of degree 7 */
    bd_down_right = 6 /* for cells of degree 7 */
    };
  #endif
  
  EX int type_of(heptagon *h) {
    return h->c7->type;
    }

  // 0 - central, -1 - left, +1 - right
  EX int mapside(heptagon *h) {
    return h->zebraval;
    }
  
  #if DEBUG_BINARY_TILING
  map<heptagon*, long long> xcode;
  map<long long, heptagon*> rxcode;

  long long expected_xcode(heptagon *h, int d) {
    auto r =xcode[h];
    if(d == 0) return r + 1;
    if(d == 1) return 2*r + 1;
    if(d == 2) return 2*r;
    if(d == 3) return 2*r - 1;
    if(d == 4) return r-1;
    if(d == 5 && type_of(h) == 6) return r / 2;
    if(d == 5 && type_of(h) == 7) return (r-1) / 2;
    if(d == 6 && type_of(h) == 7) return (r+1) / 2;
    breakhere();
    }
  #endif
  
  EX heptagon *path(heptagon *h, int d, int d1, std::initializer_list<int> p) {
    static int rec = 0;
    rec++; if(rec>100) exit(1);
    // printf("{generating path from %p (%d/%d) dir %d:", h, type_of(h), mapside(h), d);
    heptagon *h1 = h;
    for(int d0: p) {
      // printf(" [%d]", d0);
      h1 = currentmap->may_create_step(h1, d0);
      // printf(" %p", h1);
      }

    #if DEBUG_BINARY_TILING    
    if(xcode[h1] != expected_xcode(h, d)) {
      printf("expected_xcode mismatch\n");
      breakhere();
      }
    #endif
    // printf("}\n");
    if(h->move(d) && h->move(d) != h1) {
      printf("already connected to something else (1)\n");
      breakhere();
      }
    if(h1->move(d1) && h1->move(d1) != h) {
      printf("already connected to something else (2)\n");
      breakhere();
      }
    h->c.connect(d, h1, d1, false);
    rec--;
    return h1;
    }

  EX heptagon *pathc(heptagon *h, int d, int d1, std::vector<std::initializer_list<int>> p) {
    h->cmove(S7-1);
    int z = h->c.spin(S7-1);
    return path(h, d, d1, p[z]);
    }
    
  EX ld hororec_scale = 0.25;
  EX ld horohex_scale = 0.6;
  
  EX void make_binary_lands(heptagon *parent, heptagon *h) {
    if(!parent->emeraldval) parent->emeraldval = currentmap->gamestart()->land;
    eLand z = eLand(parent->emeraldval);
    int chance = 0;
    if(ls::no_walls() || parent->emeraldval == laCrossroads4) {
      eLand x = parent->c7->land;
      parent->c7->land = z;
      chance = wallchance(parent->c7, deep_ocean_at(parent->c7, parent->c7));
      parent->c7->land = x;
      }
    if(ls::std_chaos()) chance = 1000;
    if(chance && hrand(40000) < chance)
      h->emeraldval = getNewLand(z);
    else
      h->emeraldval = z;
    }
  
  EX heptagon *build(heptagon *parent, int d, int d1, int t, int side, int delta) {
    auto h = buildHeptagon1(init_heptagon(t), parent, d, hsA, d1);
    h->distance = parent->distance + delta;
    h->dm4 = parent->dm4 + delta;
    h->c7 = NULL;
    if(parent->c7) h->c7 = newCell(t, h);
    h->zebraval = side;
    switch(geometry) {
      case gBinary4:
        if(d < 2)
          h->emeraldval = gmod(parent->emeraldval * 2 + d, 15015);
        else
          h->emeraldval = gmod((parent->emeraldval - d1) * 7508, 15015);
        break;
      case gTernary:
        if(d <= 2)
          h->emeraldval = gmod(parent->emeraldval * 3 + d, 10010);
        else
          h->emeraldval = gmod((parent->emeraldval - d1) * 3337, 10010);
        break;
      case gHoroRec: {
        int x = parent->fieldval & 4095;
        int y = (parent->fieldval >> 12) & 4095;
        if(d < 2) tie(x, y) = make_pair(y, gmod(x * 2 + d, 1155));
        else tie(x,y) = make_pair(gmod((y-d1)*578, 1155), x);
        h->fieldval = x + (y << 12);
        break;
        }
      case gBinary3: {
        int x = parent->fieldval & 4095;
        int y = (parent->fieldval >> 12) & 4095;
        if(d < 4) x = gmod(x * 2 + (d&1), 1155), y = gmod(y * 2 + (d>>1), 1155);
        else x = gmod((x-(d1&1))*578, 1155), y = gmod((y-(d1>>1))*578, 1155);
        h->fieldval = x + (y << 12);
        break;
        }
      default:
        break;
      }
    if(WDIM == 3 && h->c7) make_binary_lands(parent, h);
    #if DEBUG_BINARY_TILING
    xcode[h] = expected_xcode(parent, d);
    if(rxcode.count(xcode[h])) {
      printf("xcode clash\n");
      breakhere();
      }
    rxcode[xcode[h]] = h;
    #endif
    return h;
    }

  #if MAXMDIM==4
  EX heptagon *build3(heptagon *parent, int d, int d1, int delta) {
    int side = 0;
    if(geometry == gBinary3) {
      if(d < 4) side = (parent->zebraval * 2 + d) % 5;
      if(d == S7-1) side = ((5+parent->zebraval-d1) * 3) % 5;
      }
    if(geometry == gHoroHex) {
      if(d < 3) side = (parent->zebraval + d) % 3;
      if(d == S7-1) side = (parent->zebraval + 3 - d1) % 3;
      }
    return build(parent, d, d1, S7, side, delta);
    }
  #endif
  
  struct hrmap_binary : hrmap {
  
    heptagon *origin;
    std::mt19937 directions_generator;
  
    hrmap_binary(heptagon *o) : origin(o) { set_seed(); }
    
    void set_seed() { directions_generator.seed(137137137); }
    
    int nextdir(int choices) { return directions_generator() % choices; }

    heptagon *getOrigin() override { return origin; }

    hrmap_binary() { 
      set_seed();
      origin = hyperbolic_origin();
      #if DEBUG_BINARY_TILING
      bt::xcode.clear();
      bt::rxcode.clear();
      bt::xcode[&h] = (1 << 16);
      bt::rxcode[1<<16] = &h;
      #endif
      origin->zebraval = 0;
      origin->emeraldval = 0;
      }

    heptagon *create_step(heptagon *parent, int d) override {
      auto h = parent;
      switch(geometry) {
        case gBinaryTiling: {
          switch(d) {
            case bd_right: {
              if(mapside(h) > 0 && type_of(h) == 7) 
                return path(h, d, bd_left, {bd_left, bd_down, bd_right, bd_up});
              else if(mapside(h) >= 0) 
                return build(parent, bd_right, bd_left, type_of(parent) ^ 1, 1, 0);
              else if(type_of(h) == 6)
                return path(h, d, bd_left, {bd_down, bd_right, bd_up, bd_left});
              else
                return path(h, d, bd_left, {bd_down_right, bd_up});
              }
            case bd_left: {
              if(mapside(h) < 0 && type_of(h) == 7) 
                return path(h, d, bd_right, {bd_right, bd_down, bd_left, bd_up});
              else if(mapside(h) <= 0) 
                return build(parent, bd_left, bd_right, type_of(parent) ^ 1, -1, 0);
              else if(type_of(h) == 6)
                return path(h, d, bd_right, {bd_down, bd_left, bd_up, bd_right});
              else
                return path(h, d, bd_right, {bd_down_left, bd_up});
              }
            case bd_up_right: {
              return path(h, d, bd_down_left, {bd_up, bd_right});
              }
            case bd_up_left: {
              return path(h, d, bd_down_right, {bd_up, bd_left});
              }
            case bd_up: 
              return build(parent, bd_up, bd_down, 6, mapside(parent), 1);
            default:
              /* bd_down */
              if(type_of(h) == 6) {
                if(mapside(h) == 0)
                  return build(parent, bd_down, bd_up, 6, 0, -1);
                else if(mapside(h) == 1)
                  return path(h, d, bd_up, {bd_left, bd_left, bd_down, bd_right});
                else if(mapside(h) == -1)
                  return path(h, d, bd_up, {bd_right, bd_right, bd_down, bd_left});
                }
              /* bd_down_left */
              else if(d == bd_down_left) {
                return path(h, d, bd_up_right, {bd_left, bd_down});
                }
              else if(d == bd_down_right) {
                return path(h, d, bd_up_left, {bd_right, bd_down});
                }
              }
          throw hr_exception("wrong dir");
          }
        case gBinary4: {
          switch(d) {
            case 0: case 1:
              return build(parent, d, 3, 5, d, 1);
            case 3:
              return build(parent, 3, parent->zebraval, 5, nextdir(2), -1);
            case 2:
              if(parent->zebraval == 0)
                return path(h, 2, 4, {3, 1});
              else
                return path(h, 2, 4, {3, 2, 0});
            case 4:
              if(parent->zebraval == 1)
                return path(h, 4, 2, {3, 0});
              else
                return path(h, 4, 2, {3, 4, 1});
            default:
              throw hr_exception("wrong dir");
            }
          }
        case gTernary: {
          switch(d) {
            case 0: case 1: case 2:
              return build(parent, d, 4, 6, d, 1);
            case 4:
              return build(parent, 4, parent->zebraval, 6, nextdir(3), -1);
            case 3:
              if(parent->zebraval < 2)
                return path(h, 3, 5, {4, parent->zebraval + 1});
              else
                return path(h, 3, 5, {4, 3, 0});
            case 5:
              if(parent->zebraval > 0)
                return path(h, 5, 3, {4, parent->zebraval - 1});
              else
                return path(h, 5, 3, {4, 5, 2});
            default:
              throw hr_exception("wrong dir");
            }
          }
        #if MAXMDIM >= 4         
        case gBinary3: {
          switch(d) {
            case 0: case 1:
            case 2: case 3:
              return build3(parent, d, 8, 1);
            case 8:
              return build3(parent, 8, nextdir(4), -1);
            case 4:
              parent->cmove(8);
              if(parent->c.spin(8) & 1)
                return path(h, 4, 5, {8, parent->c.spin(8) ^ 1});
              else
                return path(h, 4, 5, {8, 4, parent->c.spin(8) ^ 1});
            case 5:
              parent->cmove(8);
              if(!(parent->c.spin(8) & 1))
                return path(h, 5, 4, {8, parent->c.spin(8) ^ 1});
              else
                return path(h, 5, 4, {8, 5, parent->c.spin(8) ^ 1});
            case 6:
              parent->cmove(8);
              if(parent->c.spin(8) & 2)
                return path(h, 6, 7, {8, parent->c.spin(8) ^ 2});
              else
                return path(h, 6, 7, {8, 6, parent->c.spin(8) ^ 2});
            case 7:
              parent->cmove(8);
              if(!(parent->c.spin(8) & 2))
                return path(h, 7, 6, {8, parent->c.spin(8) ^ 2});
              else
                return path(h, 7, 6, {8, 7, parent->c.spin(8) ^ 2});
            default:
              throw hr_exception("wrong dir");
            }
          }
        case gHoroRec: {
          switch(d) {
            case 0: case 1:
              return build3(parent, d, 6, 1);
            case 6:
              return build3(parent, 6, nextdir(2), -1);
            case 2:
              parent->cmove(6);
              if(parent->c.spin(6) == 0)
                return path(h, 2, 4, {6, 1});
              else
                return path(h, 2, 4, {6, 3, 0});
            case 4:
              parent->cmove(6);
              if(parent->c.spin(6) == 0)
                return path(h, 4, 2, {6, 5, 1});
              else
                return path(h, 4, 2, {6, 0});
            case 3:
              parent->cmove(6);
              return path(h, 3, 5, {6, 4, parent->c.spin(6)});
            case 5:
              parent->cmove(6);
              return path(h, 5, 3, {6, 2, parent->c.spin(6)});
            default:
              throw hr_exception("wrong dir");
            }
          }
        case gHoroTris: {            
          switch(d) {
            case 0: case 1: case 2: case 3:
              return build3(parent, d, 7, 1);
            case 7:
              return build3(parent, 7, nextdir(3), -1);
            case 4: case 5: case 6: {
              parent->cmove(7);
              int s = parent->c.spin(7);
              if(s == 0) return path(h, d, d, {7, d-3});
              else if(s == d-3) return path(h, d, d, {7, 0});
              else return path(h, d, d, {7, d, 9-d-s});
              }
            default:
              throw hr_exception("wrong dir");
            }
          }
        case gHoroHex: {
          // the comment is a picture...
          // generated with the help of hexb.cpp          
          switch(d) {
            case 0: case 1: case 2:
              return build3(parent, d, 13, 1);
            case 13:
              return build3(parent, 13, nextdir(3), -1);
            case 3:
              return pathc(h, 3, 12, {{13,4,2}, {13,5,2}, {13,3,2}});
            case 4:
              return pathc(h, 4, 12, {{13,6,2,0}, {13,7,0,0}, {13,8,1,0}});
            case 5:
              return pathc(h, 5, 12, {{13,1,1}, {13,2,1}, {13,0,1}});
            case 6:
              return pathc(h, 6, 10, {{13,5}, {13,3}, {13,4}});
            case 7:
              return pathc(h, 7, 11, {{13,2}, {13,0}, {13,1}});
            case 8:
              return pathc(h, 8, 9, {{13,6,0}, {13,7,1}, {13,8,2}});
            case 9:
              return pathc(h, 9, 8, {{13,4}, {13,5}, {13,3}});
            case 10:
              return pathc(h, 10, 6, {{13,6,2}, {13,7,0}, {13,8,1}});
            case 11:
              return pathc(h, 11, 7, {{13,1}, {13,2}, {13,0}});
            case 12: {
              h->cmove(13);
              int z = h->c.spin(13);
              return path(h, 12, (z+1)%3+3, {13, z+6});
              }
            default:
              throw hr_exception("wrong dir");
            }
          }
        #endif
        default: 
          throw hr_exception("wrong geometry");
        }
      }

    int shvid(cell *c) override {
      if(geometry == gBinaryTiling)  
        return c->type-6;
      else if(geometry == gBinary4 || geometry == gTernary)
        return c->master->zebraval;
      else
        return 0;
      }

    hyperpoint get_corner(cell *c, int cid, ld cf) override {
      if(WDIM == 3) {
        println(hlog, "get_corner_position called");
        return C0;
        }
      return mid_at_actual(bt::get_horopoint(bt::get_corner_horo_coordinates(c, cid)), 3/cf);
      }

    int updir_at(heptagon *h) {
      if(geometry != gBinaryTiling) return updir();
      else if(type_of(h) == 6) return bd_down;
      else if(mapside(h) == 1) return bd_left;
      else if(mapside(h) == -1) return bd_right;
      else throw hr_exception("wrong dir");
      }

    transmatrix relative_matrixh(heptagon *h2, heptagon *h1, const hyperpoint& hint) override {
      if(gmatrix0.count(h2->c7) && gmatrix0.count(h1->c7))
        return inverse_shift(gmatrix0[h1->c7], gmatrix0[h2->c7]);
      transmatrix gm = Id, where = Id;
      while(h1 != h2) {
        if(h1->distance <= h2->distance) {
          int d = updir_at(h2);
          where = iadj(h2, d) * where;
          h2 = may_create_step(h2, d);
          }
        else {
          int d = updir_at(h1);
          gm = gm * adj(h1, d);
          h1 = may_create_step(h1, d);
          }
        }
      return gm * where;
      }

    ld spin_angle(cell *c, int d) override {
      if(WDIM == 3 || geometry == gBinary4 || geometry == gTernary) {
        return hrmap::spin_angle(c, d);
        }
      if(d == NODIR) return 0;
      if(d == c->type-1) d++;
      return -(d+2) * 45._deg;
      }

    transmatrix adj(heptagon *h, int dir) override {
      if(geometry == gBinaryTiling) switch(dir) {
        case bd_up: return lxpush(-log(2));
        case bd_left: return parabolic(-2);
        case bd_right: return parabolic(+2);
        case bd_down: 
          if(h->type == 6) return lxpush(log(2));
          /* case bd_down_left: */
          return parabolic(-2) * lxpush(log(2));
        case bd_down_right: 
          return parabolic(+2) * lxpush(log(2));
        case bd_up_left:
          return lxpush(-log(2)) * parabolic(-2);
        case bd_up_right:
          return lxpush(-log(2)) * parabolic(2);
        default:
          throw hr_exception("unknown direction");
        }
      else if(use_direct_for(dir))
        return cgi.direct_tmatrix[dir];
      else {
        h->cmove(dir);
        return cgi.inverse_tmatrix[h->c.spin(dir)];
        }
      }

    transmatrix iadj(heptagon *h, int dir) { heptagon *h1 = h->cmove(dir); return adj(h1, h->c.spin(dir)); }
  
    void virtualRebase(heptagon*& base, transmatrix& at) override {
    
      while(true) {
      
        double currz = at[LDIM][LDIM];
        
        heptagon *h = base;
        
        heptagon *newbase = NULL;
        
        transmatrix bestV;
        
        for(int d=0; d<S7; d++) {
          transmatrix V2 = iadj(h, d) * at;
          double newz = V2[LDIM][LDIM];
          if(newz < currz) {
            currz = newz;
            bestV = V2;
            newbase = h->cmove(d);
            }
          }
    
        if(newbase) {
          base = newbase;
          at = bestV;
          continue;
          }
    
        return;
        }
      }

    ~hrmap_binary() { if(origin) clearfrom(origin); }
    };

  EX hrmap *new_map() { return new hrmap_binary; }
  
  struct hrmap_alternate_binary : hrmap_binary {
    heptagon *origin;
    hrmap_alternate_binary(heptagon *o) { origin = o; }
    ~hrmap_alternate_binary() { clearfrom(origin); }
    };

  EX hrmap *new_alt_map(heptagon *o) { return new hrmap_binary(o); }

  /** \brief return if ew should use direct_tmatrix[dir] to get the adjacent cell the given direction
   *  
   *  Otherwise, this is the 'up' direction and thus we should use inverse_tmatrix for the inverse direction
   */
  EX bool use_direct_for(int dir) {
    return (cgi.use_direct >> dir) & 1;
    }

  /** \brief which coordinate is expanding */
  EX int expansion_coordinate() {
    if(WDIM == 2) return 0;
    return 2;
    }
  
  /** \brief by what factor does the area expand after moving one level in hr::bt::expansion_coordinate() */
  EX ld area_expansion_rate() {
    switch(geometry) {
      case gBinaryTiling: case gBinary4:
        return 2;
      case gTernary:
        return 3;
      case gBinary3: case gHoroTris:
        return 4;
      case gHoroRec:
        return 2;
      case gHoroHex:
        return 3;
      case gNil:
        return 1;
      case gEuclidSquare:
        return 1;
      case gKiteDart3:
        return pow(golden_phi, 2);
      case gSol:
        return 1;
      case gNIH:
        return 6;
      case gSolN:
        return 3/2.;
      case gArnoldCat:
        return 1;
      
      default:
        return 0;
      }
    }
  
  /** \brief by what factor do the lengths expand after moving one level in hr::bt::expansion_coordinate() */
  EX ld expansion() {
    if(WDIM == 2) return area_expansion_rate();
    else if(mproduct) return PIU( area_expansion_rate() );
    else return sqrt(area_expansion_rate());
    }
  
  /** \brief Get a point in the current cell, normalized to [-1,1]^WDIM
   *
   *  This function returns the matrix moving point (0,0,0) to the given point in a parallelogram-like box
   *  Dimensions of the box are normalized to [-1,1], and directions are the same as usual (i.e., expansion_coordinate() is the correct one)
   *  
   *  This should works for all geometries which actually have boxes.
   *
   *  For binary-based tessellations which are not based on square sections (e.g. gKiteDart3), 'x' and 'y' coordinates are not given in [-1,1], but take binary_width into account
   *
   *  Otherwise: just return h
   *
   *  See also: in devmods/tests.cpp, -bt-test tests whether this works correctly
   *
   */

  EX transmatrix normalized_at(hyperpoint h) {
    ld z2 = -log(2) / 2;
    ld z3 = -log(3) / 2;
    ld bwhn = vid.binary_width / 2;
    ld bwh = vid.binary_width * z2;
    ignore(bwh); ignore(bwhn);
    ld r2 = sqrt(2);
    const ld hs = hororec_scale;
    auto &x = h[0], &y = h[1], &z = h[2];
    switch(geometry) {
      case gBinaryTiling: case gBinary4:
        return bt::parabolic(y) * lxpush(x*z2*2);
      case gTernary:
        return bt::parabolic(y) * lxpush(x*z3*2);
      #if CAP_SOLV
      case gSol:
        return xpush(bwh*x) * ypush(bwh*y) * zpush(z2*z);
      case gSolN: case gNIH:
        return xpush(bwhn*x) * ypush(bwhn*y) * zpush(-z*.5);
      case gArnoldCat:
        return rgpushxto0(asonov::tx*x/2 + asonov::ty*y/2 + asonov::tz*z/2);
      #endif
      case gNil:
        return rgpushxto0(point31(x/2, y/2, z/2));
      case gEuclidSquare:
        return rgpushxto0(hpxy(x, y));
      case gBinary3:
        return parabolic3(x,y) * xpush(z*z2);
      case gHoroRec:
        return parabolic3(r2*hs*x, 2*hs*y) * xpush(z*z2/2);
      case gHoroTris: 
        return parabolic3(x,y) * xpush(z*z2);
      case gHoroHex: 
        return parabolic3(x,y) * xpush(z*z3/2);
      case gKiteDart3:
        return parabolic3(x,y) * xpush(-z*log_golden_phi/2);
      default:
        return rgpushxto0(h);
      }
    }
  
  EX transmatrix normalized_at(ld x, ld y, ld z IS(0)) {
    return normalized_at(point3(x, y, z));
    }
  
  EX int updir() {
    if(geometry == gBinary4) return 3;
    if(geometry == gTernary) return 4;
    if(geometry == gBinaryTiling) return 5;
    if(kite::in()) return 0;
    if(!bt::in()) return 0;
    return S7-1;
    }
  
  EX int dirs_outer() {
    switch(geometry) {
      case gBinary3: return 4;
      case gHoroTris: return 4;
      case gHoroRec: return 2;
      case gHoroHex: return 6;
      default: return -1;
      }
    }

  EX int dirs_inner() {
    if(among(geometry, gBinaryTiling, gHoroHex)) return 2;
    return 1;
    }
  
  EX void build_tmatrix() {
    if(among(geometry, gBinaryTiling, gSol, gArnoldCat)) return; // unused
    auto& direct_tmatrix = cgi.direct_tmatrix;
    auto& inverse_tmatrix = cgi.inverse_tmatrix;
    auto& use_direct = cgi.use_direct;
    use_direct = (1 << (S7-1)) - 1;    
    if(geometry == gBinary4) {
      use_direct = 3;
      direct_tmatrix[0] = lxpush(-log(2)) * parabolic(-1);
      direct_tmatrix[1] = lxpush(-log(2)) * parabolic(+1);
      direct_tmatrix[2] = parabolic(2);
      direct_tmatrix[4] = parabolic(-2);
      use_direct = 1+2+4+16;
      }
    if(geometry == gTernary) {
      direct_tmatrix[0] = lxpush(-log(3)) * parabolic(-2);
      direct_tmatrix[1] = lxpush(-log(3));
      direct_tmatrix[2] = lxpush(-log(3)) * parabolic(+2);
      direct_tmatrix[3] = parabolic(2);
      direct_tmatrix[5] = parabolic(-2);
      use_direct = 1+2+4+8+32;
      }
    if(geometry == gBinary3) {
      direct_tmatrix[0] = xpush(-log(2)) * parabolic3(-1, -1);
      direct_tmatrix[1] = xpush(-log(2)) * parabolic3(1, -1);
      direct_tmatrix[2] = xpush(-log(2)) * parabolic3(-1, 1);
      direct_tmatrix[3] = xpush(-log(2)) * parabolic3(1, 1);
      direct_tmatrix[4] = parabolic3(-2, 0);
      direct_tmatrix[5] = parabolic3(+2, 0);
      direct_tmatrix[6] = parabolic3(0, -2);
      direct_tmatrix[7] = parabolic3(0, +2);
      }
    if(geometry == gHoroTris) {
      ld r3 = sqrt(3);
      direct_tmatrix[0] = xpush(-log(2)) * cspin180(1,2);
      direct_tmatrix[1] = parabolic3(0, +r3/3) * xpush(-log(2));
      direct_tmatrix[2] = parabolic3(-0.5, -r3/6) * xpush(-log(2));
      direct_tmatrix[3] = parabolic3(+0.5, -r3/6) * xpush(-log(2));
      direct_tmatrix[4] = parabolic3(0, -r3*2/3) * cspin180(1,2);
      direct_tmatrix[5] = parabolic3(1, r3/3) * cspin180(1,2);
      direct_tmatrix[6] = parabolic3(-1, r3/3) * cspin180(1,2);
      }
    if(geometry == gHoroRec) {
      ld r2 = sqrt(2);
      ld l = -log(2)/2;
      ld z = hororec_scale;
      direct_tmatrix[0] = parabolic3(0, -z) * xpush(l) * cspin90(2,1);
      direct_tmatrix[1] = parabolic3(0, +z) * xpush(l) * cspin90(2,1);
      direct_tmatrix[2] = parabolic3(+2*r2*z, 0);
      direct_tmatrix[3] = parabolic3(0, +4*z);
      direct_tmatrix[4] = parabolic3(-2*r2*z, 0);
      direct_tmatrix[5] = parabolic3(0, -4*z);
      }
    if(geometry == gHoroHex) {
      // also generated with the help of hexb.cpp
      ld l = log(3)/2;
      auto& t = direct_tmatrix;
      t[0] = parabolic3(horohex_scale, 0) * xpush(-l) * cspin(1, 2, 90._deg);
      t[1] = cspin(1, 2, 120*degree) * t[0];
      t[2] = cspin(1, 2, 240*degree) * t[0];
      auto it = iso_inverse(t[0]);

      t[5] = it * t[1] * t[1];
      t[6] = it * t[5];
      t[4] = it * t[6] * t[2] * t[0];
      t[3] = it * t[4] * t[2];

      t[7] = it * t[2];
      t[8] = it * t[6] * t[0];
      t[9] = it * t[4];
      t[10] = it * t[6] * t[2];
      t[11] = it * t[1];

      if(debugflags & DF_GEOM)
        for(int a=0; a<12; a++) 
          println(hlog, t[a]);

      use_direct >>= 1;
      }
    for(int i=0; i<S7; i++) if(use_direct_for(i))
      inverse_tmatrix[i] = iso_inverse(direct_tmatrix[i]);
    }
  
  #if MAXMDIM == 4

  EX void queuecube(const shiftmatrix& V, ld size, color_t linecolor, color_t facecolor) {
    ld yy = log(2) / 2;
    const int STEP=3;
    const ld MUL = 1. / STEP;
    auto at = [&] (ld x, ld y, ld z) { curvepoint(parabolic3(size*x, size*y) * xpush0(size*yy*z)); };
    for(int a:{-1,1}) {
      for(ld t=-STEP; t<STEP; t++) at(a, 1,t*MUL);
      for(ld t=-STEP; t<STEP; t++) at(a, -t*MUL,1);
      for(ld t=-STEP; t<STEP; t++) at(a, -1,-t*MUL);
      for(ld t=-STEP; t<STEP; t++) at(a, t*MUL,-1);
      at(a, 1,-1);
      queuecurve(V, linecolor, facecolor, PPR::LINE);

      for(ld t=-STEP; t<STEP; t++) at(1,t*MUL,a);
      for(ld t=-STEP; t<STEP; t++) at(-t*MUL,1,a);
      for(ld t=-STEP; t<STEP; t++) at(-1,-t*MUL,a);
      for(ld t=-STEP; t<STEP; t++) at(t*MUL,-1,a);
      at(1,-1,a);
      queuecurve(V, linecolor, facecolor, PPR::LINE);

      for(ld t=-STEP; t<STEP; t++) at(1,a,t*MUL);
      for(ld t=-STEP; t<STEP; t++) at(-t*MUL,a,1);
      for(ld t=-STEP; t<STEP; t++) at(-1,a,-t*MUL);
      for(ld t=-STEP; t<STEP; t++) at(t*MUL,a,-1);
      at(1,a,-1);
      queuecurve(V, linecolor, facecolor, PPR::LINE);
      }
    /*for(int a:{-1,1}) for(int b:{-1,1}) for(int c:{-1,1}) {
      at(0,0,0); at(a,b,c);  queuecurve(linecolor, facecolor, PPR::LINE);
      }*/
    }
  #endif

  EX ld xy_mul() { return vid.binary_width * log(2) / 2; }

  EX transmatrix parabolic(ld u) {
    return parabolic1(u * xy_mul());
    }

  EX transmatrix parabolic3(ld y, ld z) {
    ld co = xy_mul();
    return hr::parabolic13(y * co, z * co);
    }

  // on which horocycle are we
  EX ld horo_level(hyperpoint h) {
    h /= (1 + h[LDIM]);
    h[0] -= 1;
    h /= sqhypot_d(GDIM, h);
    h[0] += .5;
    return log(2) + log(-h[0]);
    }
  
  /** \brief convert BT coordinates to Minkowski coordinates 
      in the BT coordinates, h[2] is vertical; the center of the horosphere in Klein model is (1,0,0) 
      */

  EX hyperpoint bt_to_minkowski(hyperpoint h) {
    ld yy = log(2) / 2;
    ld co = xy_mul();
    return hr::parabolic13(h[0] * co, h[1] * co) * xpush0(yy*h[2]);
    }

  /** \brief inverse of bt_to_minkowski */
  EX hyperpoint minkowski_to_bt(hyperpoint h) {
    h = deparabolic13(h);
    ld co = xy_mul();
    ld yy = log(2) / 2;
    h = point31(h[1] / co, h[2] / co, h[0] / yy);
    return h;
    }

#if CAP_COMMANDLINE  
auto bt_config = arg::add2("-btwidth", [] {arg::shift_arg_formula(vid.binary_width); });
#endif

EX bool pseudohept(cell *c) {
  switch(geometry) {
    case gBinary4:
      c->cmove(3);
      return (c->master->distance & 1) && (c->c.spin(3) == 0);

    case gBinaryTiling:
      return c->master->distance & c->type & 1;

    case gTernary: {
      return c->master->emeraldval & 1;
      /* auto m = dynamic_cast<hrmap_binary*> (current_map());
      auto o = m->origin;
      int flips = 0;
      while(m != o) {
        if(m->master->distance >= o->master->distance) { if(m->c.spin(4) == 1) flips++; m = m->cmove(4); }
        }
    heptagon *origin;
      c->cmove(4); return (c->c.spin(4) == 1); */
      }

    case gHoroRec:
      return c->c.spin(S7-1) == 0 && (c->master->distance & 1) && c->cmove(S7-1)->c.spin(S7-1) == 0;

    case gHoroTris:
      return c->c.spin(S7-1) == 0 && (c->master->distance & 1);

    default:
      return (c->master->zebraval == 1) && (c->master->distance & 1);
    }
  }

EX pair<gp::loc, gp::loc> gpvalue(heptagon *h) {
  int d = h->c.spin(S7-1);
  if(d == 0) return make_pair(gp::loc(0,0), gp::loc(-1,0));
  else return make_pair(gp::eudir((d-1)*2), gp::loc(1,0));
  }

// distance in a triangular grid
EX int tridist(gp::loc v) {
  using namespace gp;
  int d = v.first - v.second;
  int d0 = d % 3;
  if(d0 == 1 || d0 == -2) return 1 + min(tridist(v - eudir(0)), min(tridist(v - eudir(2)), tridist(v - eudir(4))));
  if(d0 == 2 || d0 == -1) return 1 + min(tridist(v + eudir(0)), min(tridist(v + eudir(2)), tridist(v + eudir(4))));
  return length(v * loc(1,1)) * 2 / 3;
  }

EX int equalize(heptagon*& c1, heptagon*& c2) {
  int steps = 0;
  int d1 = c1->distance;
  int d2 = c2->distance;
  while(d1 > d2) c1 = c1->cmove(S7-1), steps++, d1--;
  while(d2 > d1) c2 = c2->cmove(S7-1), steps++, d2--;
  return steps;
  }  

EX int celldistance3_tri(heptagon *c1, heptagon *c2) {
  using namespace gp;
  int steps = equalize(c1, c2);
  vector<pair<loc, loc> > m1, m2;
  while(c1 != c2) {
    m2.push_back(gpvalue(c2));
    m1.push_back(gpvalue(c1));
    c1 = c1->cmove(S7-1);
    c2 = c2->cmove(S7-1);
    steps += 2;
    }
  loc T1(0,0), T2(0,0), inv1(1,0), inv2(1,0);
  int xsteps = steps;
  while(isize(m1)) {
    xsteps -= 2;
    inv1 = inv1 * m1.back().second;
    inv2 = inv2 * m2.back().second;
    T1 = T1 + T1 + m1.back().first * inv1;
    T2 = T2 + T2 + m2.back().first * inv2;
    m1.pop_back(); m2.pop_back();
    loc T0 = T2 - T1;
    if(T0.first > 3 || T0.second > 3 || T0.first < -3 || T0.second < -3) break;
    steps = min(steps, xsteps + tridist(T0));
    }
  return steps;
  }

EX int celldistance3_rec(heptagon *c1, heptagon *c2) {
  int steps = equalize(c1, c2);
  vector<int> dx;
  while(c1 != c2) {
    dx.push_back(c1->c.spin(S7-1) - c2->c.spin(S7-1));
    c1 = c1->cmove(S7-1);
    c2 = c2->cmove(S7-1);
    steps += 2;
    }
  int xsteps = steps, sx = 0, sy = 0;
  while(isize(dx)) {
    xsteps -= 2;
    tie(sx, sy) = make_pair(-sy, 2 * sx + dx.back());
    dx.pop_back();
    int ysteps = xsteps + abs(sx) + abs(sy);
    if(ysteps < steps) steps = ysteps;
    if(sx >= 8 || sx <= -8 || sy >= 8 || sy <= -8) break;
    }
  return steps;
  }

EX int celldistance3_square(heptagon *c1, heptagon *c2) {
  int steps = equalize(c1, c2);
  vector<int> dx, dy;
  while(c1 != c2) {
    dx.push_back((c1->c.spin(S7-1) & 1) - (c2->c.spin(S7-1) & 1));
    dy.push_back((c1->c.spin(S7-1) >> 1) - (c2->c.spin(S7-1) >> 1));
    c1 = c1->cmove(S7-1);
    c2 = c2->cmove(S7-1);
    steps += 2;
    }
  int xsteps = steps, sx = 0, sy = 0;
  while(isize(dx)) {
    xsteps -= 2;
    sx *= 2;
    sy *= 2;
    sx += dx.back(); sy += dy.back();
    dx.pop_back(); dy.pop_back();
    int ysteps = xsteps + abs(sx) + abs(sy);
    if(ysteps < steps) steps = ysteps;
    if(sx >= 8 || sx <= -8 || sy >= 8 || sy <= -8) break;
    }
  return steps;
  }

// this algorithm is wrong: it never considers the "narrow gap" moves
EX int celldistance3_hex(heptagon *c1, heptagon *c2) {
  int steps = equalize(c1, c2);
  vector<int> d1, d2;
  while(c1 != c2) {
    d1.push_back(c1->c.spin(S7-1));
    d2.push_back(c2->c.spin(S7-1));
    c1 = c1->cmove(S7-1);
    c2 = c2->cmove(S7-1);
    steps += 2;
    }
  int xsteps = steps;
  dynamicval<eGeometry> g(geometry, gEuclid);
  transmatrix T = Id;
  while(isize(d1)) {
    xsteps -= 2;

    T = euscalezoom(hpxy(0,sqrt(3))) * eupush(1,0) * spin(-d2.back() * 120._deg) * T * spin(d1.back() * 2 * M_PI/3) * eupush(-1,0) * euscalezoom(hpxy(0,-1/sqrt(3)));
    
    d1.pop_back(); d2.pop_back();
    
    hyperpoint h = tC0(T);
    int sx = int(floor(h[0] - h[1] / sqrt(3) + .5)) / 3;
    int sy = int(floor(h[1] * 2 / sqrt(3) + .5)) / 3;
    
    int ysteps = xsteps + euc::dist(sx, sy);
    if(ysteps < steps) steps = ysteps;
    if(sx >= 8 || sx <= -8 || sy >= 8 || sy <= -8) break;
    }
  return steps;
  }

EX int celldistance3_approx(heptagon *c1, heptagon *c2) {
  int d = 0;
  while(true) {
    if(d > 1000000) return d; /* sanity check */
    if(c1 == c2) return d;
    for(int i=0; i<c1->type; i++)
      if(c1->move(i) == c2) return d + 1;
    for(int i=0; i<c1->type; i++) {
      heptagon *c3 = c1->move(i);
      for(int j=0; j<c3->type; j++)
        if(c3->move(j) == c2) return d+2;
        }
    if(c1->distance > c2->distance) c1=c1->cmove(updir()), d++;
    else c2=c2->cmove(updir()), d++;
    }
  }

EX int celldistance3(heptagon *c1, heptagon *c2) {
  switch(geometry) {
    case gBinary3: return celldistance3_square(c1, c2);
    case gHoroTris: return celldistance3_tri(c1, c2);
    case gHoroRec: return celldistance3_rec(c1, c2);
    case gHoroHex: return celldistance3_hex(c1, c2);
    default: 
      if(sol || !bt::in()) {
        println(hlog, "called celldistance3 for wrong geometry"); return 0;
        }
      return celldistance3_approx(c1, c2);
    }
  }

EX int celldistance3(cell *c1, cell *c2) { return celldistance3(c1->master, c2->master); }

EX hyperpoint get_horopoint(ld y, ld x) {
  return bt::parabolic(x*2) * lxpush(-y) * C0;
  }

EX hyperpoint get_horopoint(hyperpoint h) {
  return get_horopoint(h[0], h[1]);
  }

EX hyperpoint inverse_horopoint(hyperpoint h) {
  hyperpoint h1 = deparabolic13(h);
  h1[1] /= 2 * bt::xy_mul(); h1[0] *= -1;
  return h1;
  }

EX hyperpoint get_corner_horo_coordinates(cell *c, int i) {
  ld yy = log(2) / 2;
  ld xx = 1 / 2.;
  switch(geometry) {
    case gBinaryTiling:    
      switch(gmod(i, c->type)) {
        case 0: return point2(-yy, xx);
        case 1: return point2(yy, xx);
        case 2: return point2(yy, xx/2);
        case 3: return point2(yy, -xx/2);
        case 4: return point2(yy, -xx);
        case 5: return point2(-yy, -xx);
        case 6: return point2(-yy, 0);    
        default: return point2(0, 0);
        }
    
    case gBinary4:
      switch(gmod(i, c->type)) {
        case 0: return point2(yy, -xx);
        case 1: return point2(yy, +0*xx);
        case 2: return point2(yy, +xx);
        case 3: return point2(-yy, xx);
        case 4: return point2(-yy, -xx);
        default: return point2(0, 0);
        }
    
    case gTernary:
      yy = log(3) / 2;
      switch(gmod(i, c->type)) {
        case 0: return point2(yy, -xx);
        case 1: return point2(yy, -xx/3);
        case 2: return point2(yy, +xx/3);
        case 3: return point2(yy, +xx);
        case 4: return point2(-yy, xx);
        case 5: return point2(-yy, -xx);
        default: return point2(0, 0);
        }
    
    default:
      return point2(0, 0);
    }
  return point2(0, 0);
  }

vector<hyperpoint> make4(hyperpoint a, hyperpoint b, hyperpoint c) {
  return {a, b, b+c-a, c};
  }

vector<hyperpoint> make5(hyperpoint a, hyperpoint b, hyperpoint c) {
  return {a, (a+b)/2, b, b+c-a, c};
  }

EX void create_faces() {
  
  if(geometry == gBinary3) {
    hyperpoint h00 = point3(-1,-1,-1);
    hyperpoint h01 = point3(-1,0,-1);
    hyperpoint h02 = point3(-1,+1,-1);
    hyperpoint h10 = point3(0,-1,-1);
    hyperpoint h11 = point3(0,0,-1);
    hyperpoint h12 = point3(0,+1,-1);
    hyperpoint h20 = point3(+1,-1,-1);
    hyperpoint h21 = point3(+1,0,-1);
    hyperpoint h22 = point3(+1,+1,-1);
    hyperpoint down = point3(0,0,2);

    add_wall(0, make4(h11, h01, h10));
    add_wall(1, make4(h11, h21, h10));
    add_wall(2, make4(h11, h01, h12));
    add_wall(3, make4(h11, h21, h12));
    add_wall(4, make5(h00, h02, h00+down));
    add_wall(5, make5(h20, h22, h20+down));
    add_wall(6, make5(h00, h20, h00+down));
    add_wall(7, make5(h02, h22, h02+down));
    add_wall(8, make4(h22+down, h02+down, h20+down));
    }

  if(GDIM == 3 && bt::in() && geometry == gHoroTris) {
    ld r = sqrt(3)/6;
    ld r1 = r;
    ld r2 = r * 2;

    hyperpoint t0 = point3(0,-r2,-1);
    hyperpoint t1 = point3(+.5,r1,-1);
    hyperpoint t2 = point3(-.5,r1,-1);
    hyperpoint shift = point3(0,0,-3);
    hyperpoint down = point3(0,0,2);
    hyperpoint d0 = -2 * t0 + shift;
    hyperpoint d1 = -2 * t1 + shift;
    hyperpoint d2 = -2 * t2 + shift;

    add_wall(0, {t0, t1, t2});
    add_wall(1, {d0, t1, t2});
    add_wall(2, {t0, d1, t2});
    add_wall(3, {t0, t1, d2});
    add_wall(4, make5(d2, d1, d2 + down));
    add_wall(5, make5(d0, d2, d0 + down));
    add_wall(6, make5(d1, d0, d1 + down));
    add_wall(7, {d0+down, d1+down, d2+down});
    }

  if(geometry == gHoroRec) {
    ld r2 = sqrt(2);
    ld z = bt::hororec_scale;

    hyperpoint a00 = point3(-r2*z,-2*z,-.5);
    hyperpoint a01 = point3(+r2*z,-2*z,-.5);
    hyperpoint a10 = point3(-r2*z, 0*z,-.5);
    hyperpoint a11 = point3(+r2*z, 0*z,-.5);
    hyperpoint a20 = point3(-r2*z,+2*z,-.5);
    hyperpoint a21 = point3(+r2*z,+2*z,-.5);

    hyperpoint down = point3(0,0,1);

    add_wall(0, make4(a00, a01, a10));
    add_wall(1, make4(a10, a11, a20));
    add_wall(2, make5(a01, a21, a01+down));
    add_wall(3, make4(a21, a20, a21+down));
    add_wall(4, make5(a20, a00, a20+down));
    add_wall(5, make4(a00, a01, a00+down));
    add_wall(6, make4(a00+down, a01+down, a20+down));
    }

  if(geometry == gHoroHex) {
    ld z = log(3) / log(2) / 2;
    ld r3 = sqrt(3) / 2 * bt::horohex_scale;
    ld h = bt::horohex_scale / 2;
    hyperpoint down = point3(0,0,2*z);

    for(int j=0; j<4; j++) for(int i=0; i<3; i++) {
      transmatrix T = cspin(0, 1, 120._deg * i);

      hyperpoint hcenter = point3(0,0,-z);
      hyperpoint hu0 = T*point3(+h,  +r3,-z);
      hyperpoint hu1 = T*point3(+h*3,+r3,-z);
      hyperpoint hd0 = T*point3(+h,  -r3,-z);
      hyperpoint hd1 = T*point3(+h*3,-r3,-z);
      hyperpoint hcn = T*point3(-h*2,0,  -z);
      hyperpoint hun = T*point3(-h*3,+r3,-z);
      hyperpoint hdn = T*point3(-h*3,-r3,-z);
      if(j == 0) add_wall(i, {hcenter, hu0, hu1, hd1, hd0});
      if(j == 1) add_wall(i+3, {hcn, hun, hdn});
      if(j == 2) add_wall(i+6, make4(hd1, hu1, hd1+down));
      if(j == 3) add_wall(i+9, make4(hun, hdn, hun+down));
      }

    add_wall(12, {point3(3*h,r3,z), point3(0,2*r3,z), point3(-3*h,r3,z)});
    add_wall(13, {point3(3*h,r3,z), point3(3*h,-r3,z), point3(0,-2*r3,z), point3(-3*h,-r3,z), point3(-3*h,r3,z)});
    }

  if(kite::in()) {
    auto kv = kite::make_walls();
    for(auto& v: kv.first) for(auto& h: v) {
      h = minkowski_to_bt(h);
      }
    for(int i=0; i<isize(kv.first); i++) {
      add_wall(i, kv.first[i]);
      }
    get_hsh().weights = kv.second;
    }

  get_hsh().compute_hept();
  }

auto hooksw = addHook(hooks_swapdim, 100, [] {
  if(bt::in()) build_tmatrix();
  });
#endif

  }

}
