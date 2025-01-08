// HyperRogue
// Copyright (C) 2011-2019 Zeno Rogue, see 'hyper.cpp' for details

/** \file 3d-models.cpp 
 *  \brief This file contains the routines to convert HyperRogue's old vector graphics into 3D models
 */

#include "hyper.h"
#include "earcut.hpp"

namespace hr {

ld eyepos;

#if MAXMDIM >= 4

#define S (cgi.scalefactor / 0.805578)
#define SH (embedded_plane ? cgi.human_height : (cgi.scalefactor / 0.805578 * (vid.height_width / 1.5)))

#define revZ ((WDIM == 2 || gproduct) ? -1 : 1)

hyperpoint shcenter;

EX hyperpoint front_leg, rear_leg;
EX transmatrix front_leg_move, rear_leg_move, front_leg_move_inverse, rear_leg_move_inverse;
EX ld leg_length;

vector<hyperpoint> geometry_information::get_shape(hpcshape sh) {
  vector<hyperpoint> res;
  for(int i=sh.s; i<sh.e-1; i++) res.push_back(hpc[i]);
  return res;  
  }

hyperpoint get_center(const vector<hyperpoint>& vh) {
  hyperpoint h = Hypc;
  for(auto h1: vh) h = h + h1;
  h /= isize(vh);
  return cgi.emb->normalize_flat(h);
  }

EX ld zc(ld z) { 
  if(WDIM == 2 && GDIM == 3)
    return geom3::lev_to_factor(cgi.human_height * z);
  return cgi.human_height * (z - 0.5) * revZ;
  }

void geometry_information::add_cone(ld z0, const vector<hyperpoint>& vh, ld z1) {
  last->flags |= POLY_TRIANGLES;
  for(int i=0; i<isize(vh); i++) {
    hpcpush(lzpush(z0) * vh[i]);
    hpcpush(lzpush(z0) * vh[(i+1) % isize(vh)]);
    hpcpush(lzpush(z1) * shcenter);
    }
  }

void geometry_information::add_prism_sync(ld z0, vector<hyperpoint> vh0, ld z1, vector<hyperpoint> vh1) {
  last->flags |= POLY_TRIANGLES;
  for(int i=0; i<isize(vh0); i++) {
    int i1 = (i+1) % isize(vh0);
    hpcpush(lzpush(z0) * vh0[i]);
    hpcpush(lzpush(z1) * vh1[i]);
    hpcpush(lzpush(z0) * vh0[i1]);
    hpcpush(lzpush(z1) * vh1[i]);
    hpcpush(lzpush(z0) * vh0[i1]);
    hpcpush(lzpush(z1) * vh1[i1]);
    }
  }

void geometry_information::add_prism(ld z0, vector<hyperpoint> vh0, ld z1, vector<hyperpoint> vh1) {
  last->flags |= POLY_TRIANGLES;
  
  struct mixed {
    ld angle;
    int owner;
    hyperpoint h;
    mixed(ld a, int o, hyperpoint _h) : angle(a), owner(o), h(_h) {}
    };
  
  transmatrix T0 = gpushxto0(get_center(vh0));
  transmatrix T1 = gpushxto0(get_center(vh1));
  
  vector<mixed> pairs;
  for(auto h: vh0) pairs.emplace_back(atan2(T0*h), 0, h);
  for(auto h: vh1) pairs.emplace_back(atan2(T1*h), 1, h);
  sort(pairs.begin(), pairs.end(), [&] (const mixed p, const mixed q) { return p.angle < q.angle; });
  
  hyperpoint lasts[2];
  for(auto pp: pairs) lasts[pp.owner] = pp.h;
  
  for(auto pp: pairs) {
    int id = pp.owner;
    hpcpush(lzpush(z0) * lasts[0]);
    hpcpush(lzpush(z1) * lasts[1]);
    hpcpush(lzpush(id == 0 ? z0 : z1) * pp.h);
    lasts[id] = pp.h;
    }
  }
  
void geometry_information::shift_last(ld z) {
  for(int i=last->s; i<isize(hpc); i++) hpc[i] = lzpush(z) * hpc[i];
  }

void geometry_information::shift_shape(hpcshape& sh, ld z) {
  for(int i=sh.s; i<sh.e; i++) hpc[i] = lzpush(z) * hpc[i];
  }

void geometry_information::shift_shape_orthogonally(hpcshape& sh, ld z) {
  for(int i=sh.s; i<sh.e; i++) hpc[i] = orthogonal_move(hpc[i], z);
  }

extern renderbuffer *floor_textures;

void geometry_information::add_texture(hpcshape& sh) {
#if CAP_GL
  if(!floor_textures) return;
  auto& utt = models_texture;
  sh.tinf = &utt;
  sh.texture_offset = isize(utt.tvertices);

  auto f = [] (hyperpoint h) {
    if(!embedded_plane && gproduct) return product::inverse_exp(h);
    return cgi.emb->actual_to_logical(h);
    };

  hyperpoint ct = Hypc;
  int n = 0;
  for(int i=sh.s; i<isize(hpc); i++) ct += f(hpc[i]), n++;
  ct /= n;

  for(int i=sh.s; i<isize(hpc); i++) {
    hyperpoint h = f(hpc[i]) - ct;
    ld rad = hypot_d(3, h);
    ld factor = 0.50 + (0.17 * h[2] + 0.13 * h[1] + 0.15 * h[0]) / rad;
    utt.tvertices.push_back(glhr::makevertex(0, factor, 0));
    }
#endif
  }

vector<hyperpoint> scaleshape(const vector<hyperpoint>& vh, ld s) {
  vector<hyperpoint> res;
  for(hyperpoint h: vh) res.push_back(cgi.emb->normalize_flat(h * s + shcenter * (1-s)));
  return res;
  }

ld get_zlevel(hyperpoint h) {
  if(gproduct) return zlevel(h);
  if(sl2) return atan2(h[2], h[3]);
  return asin_auto(h[2]);
  }

void geometry_information::make_ha_3d(hpcshape& sh, bool isarmor, ld scale) {
  shcenter = tile_center();

  auto groin = get_shape(shHumanGroin);
  auto body = get_shape(shPBodyOnly);
  auto neck = get_shape(shHumanNeck);
  auto hand = get_shape(shPBodyHand);
  auto arm = get_shape(shPBodyArm);
  groin = scaleshape(groin, scale);
  neck = scaleshape(neck, scale);

  auto fullbody = get_shape(sh);
  
  auto body7 = body[7];
  auto body26 = body[26];
  body.clear();

  auto& T = cgi.emb->intermediate_to_logical;

  bool foundplus = false, foundminus = false;
  for(hyperpoint h: fullbody) {
    if((T*h)[1] > 0.14 * S) {
      if(foundplus) ;
      else foundplus = true, body.push_back(body7);
      }
    else if((T*h)[1] < -0.14 * S) {
      if(foundminus) ;
      else foundminus = true, body.push_back(body26);
      }
    else body.push_back(h);
    }
  
  auto arm8 = arm[8];
  bool armused = false;
  arm.clear();  
  for(hyperpoint h: fullbody) {
    if((T*h)[1] < 0.08 * S) ;
    else if((T*h)[0] > -0.03 * S) {
      if(armused) ;
      else armused = true, arm.push_back(arm8);
      }
    else arm.push_back(h);
    }

  auto hand0 = hand[0];
  hand.clear();
  hand.push_back(hand0);
  for(hyperpoint h: fullbody) {
    auto h1 = T*h;
    if(h1[1] + h1[0] > 0.13 * S) hand.push_back(h);
    }

  bshape(sh, PPR::MONSTER_BODY);
  add_cone(zc(0.4), groin, zc(0.36));
  add_prism_sync(zc(0.4), groin, zc(0.6), groin);
  add_prism(zc(0.6), groin, zc(0.7), body);
  add_prism(zc(0.7), body, zc(0.8), neck);
  
  add_cone(zc(0.8), neck, zc(0.83));

  int at0 = isize(hpc);
  ld h = human_height;
  
  if(isize(arm) > 3) {
    shcenter = get_center(arm);
    int arm0 = isize(hpc);
    add_prism_sync(BODY - h*.03, arm, BODY + h*.03, arm);
    add_cone(BODY + h*.03, arm, BODY + h*.05);
    add_cone(BODY - h*.03, arm, BODY - h*.05);
    int arm1 = isize(hpc);
    for(int i=arm0; i<arm1; i++) {
      hyperpoint h = hpc[i];
      ld zl = get_zlevel(h);
      h = lzpush(-zl) * h;
      ld rad = hdist0(h);
      rad = (rad - 0.1124*S) / (0.2804*S - 0.1124*S);
      rad = 1 - rad;
      rad *= zc(0.7) - BODY;
      hpc[i] = lzpush(rad) * hpc[i];
      }
    }
   // 0.2804 - keep
   // 0.1124 - move
   
  if(isize(hand) > 3) {
    shcenter = get_center(hand);
    add_cone(BODY, hand, BODY + 0.05 * human_height);
    add_cone(BODY, hand, BODY - 0.05 * human_height);
    }

  int at1 = isize(hpc);
  for(int i=at0; i<at1; i++) hpc.push_back(Mirror * hpc[i]);  
  
  add_texture(sh);
  shift_last(-BODY);
  }

void geometry_information::make_humanoid_3d(hpcshape& sh) { make_ha_3d(sh, false, 1); }

void geometry_information::addtri(array<hyperpoint, 3> hs, int kind) {
  ld ds[3];
  ds[0] = hdist(hs[0], hs[1]);
  ds[1] = hdist(hs[1], hs[2]);
  ds[2] = hdist(hs[2], hs[0]);
  ld maxds = 0;
  for(int i=0; i<3; i++) maxds = max(ds[i], maxds) - 1e-3;
  
  if(maxds > 0.02*S) for(int i=0; i<3; i++) {
    int j = (i+1) % 3;
    int k = (j+1) % 3;
    if(hdist(hs[i], hs[j]) > maxds) {
      auto hm = mid(hs[i], hs[j]);
      addtri(make_array(hm, hs[i], hs[k]), kind);
      addtri(make_array(hm, hs[j], hs[k]), kind);
      return;
      }
    }
  
  if(kind) {
    array<hyperpoint, 3> ht;
    ld hsh[3];
    ld shi[3];
    bool ok = true;
    ld zzes[3];
    for(int s=0; s<3; s++) {
      hs[s] = cgi.emb->normalize_flat(hs[s]);
      hyperpoint h = hs[s];
      ld zz = zc(0.78);
      hsh[s] = abs(h[1]);
      zz -= h[1] * h[1] / 0.14 / 0.14 * 0.01 / S / S * SH;
      zz -= h[0] * h[0] / 0.10 / 0.10 * 0.01 / S / S * SH;
      if(abs(h[1]) > 0.14*S) ok = false, zz -= revZ * (abs(h[1])/S - 0.14) * SH;
      if(abs(h[0]) > 0.08*S) ok = false, zz -= revZ * (abs(h[0])/S - 0.08) * (abs(h[0])/S - 0.08) * 25 * SH;
      h = cgi.emb->normalize_flat(h);
      if(!gproduct || kind != 1) ht[s] = lzpush(zz) * h;
      else ht[s] = h;
      if(hsh[s] < 0.1*S) shi[s] = 0.5;
      else if(hsh[s] < 0.12*S) shi[s] = 0.1 + 0.4 * (hsh[s]/S - 0.1) / (0.12 - 0.1);
      else shi[s] = 0.1;
      zzes[s] = zz;
      }
    if(ok && kind == 1) {
      array<array<hyperpoint, 3>, 6> htx;
      for(int i=0; i<6; i++) htx[i] = ht;

      for(int i=0; i<3; i++) {
        htx[0][i][0] *= 0.7; htx[0][i][1] *= 0.7;
        htx[1][i][0] *= 1.2; htx[1][i][1] *= 1.7;
        htx[2][i][1] *= 1.7;
        htx[4][i][0] = htx[4][i][0] * 0.4 + scalefactor * 0.1;
        htx[5][i][0] = htx[5][i][0] * 0.3 + scalefactor * 0.1;
        if(!gproduct)
          for(int a=0; a<6; a++) htx[a][i] = hpxy3(htx[a][i][0], htx[a][i][1], htx[a][i][2]);
        else
          for(int a=0; a<6; a++) htx[a][i] = lzpush(zzes[i]) * hpxy(htx[a][i][0], htx[a][i][1]);
        }
      ld levels[6] = {0, 0.125, 0.125, 0.250, 0.375, 0.5};
      for(int a=0; a<6; a++) for(int i=0; i<3; i++) 
        htx[a][i] = lzpush(-min(shi[i], levels[a]) * human_height * revZ) * htx[a][i];
      
      hpcpush(htx[0][0]);
      hpcpush(htx[0][1]);
      hpcpush(htx[0][2]);

      for(int a=0; a<5; a++) for(int i=0; i<3; i++) {
        int j = (i+1) % 3;
        int b = a+1;
        hpcpush(htx[a][i]);
        hpcpush(htx[a][j]);
        hpcpush(htx[b][i]);
        hpcpush(htx[a][j]);
        hpcpush(htx[b][i]);
        hpcpush(htx[b][j]);
        }
      }
    else 
      hpcpush(ht[0]), hpcpush(ht[1]), hpcpush(ht[2]);
    }
  else {
    for(int s=0; s<3; s++) {
      hyperpoint h = hs[s];
      ld zz = zc(eyepos);
      if(h[0] < -0.05*S) zz += revZ * (h[0]/S + 0.05) * SH;
      if(hdist0(h) <= 0.0501*S) {
        zz += revZ * sqrt(0.0026 - pow(hdist0(h)/S, 2)) * SH;
        }
      hpcpush(lzpush(zz) * h);
      }
    }
  }

void disable(hpcshape& sh) {
  sh.s = sh.e = 0;
  }

void geometry_information::make_armor_3d(hpcshape& sh, int kind) { 

  if(BADMODEL) {
    disable(sh);
    return;
    }
  auto body = get_shape(sh);
  vector<vector<array<ld, 2> >> pts(2);
  
  for(hyperpoint h: body) {
    array<ld, 2> p;
    p[0] = h[0] / h[LDIM];
    p[1] = h[1] / h[LDIM];
    pts[0].emplace_back(p);
    }
  
  bshape(sh, sh.prio);
  
  vector<int> indices = mapbox::earcut<int> (pts);
  
  last->flags |= POLY_TRIANGLES;

  last->flags |= POLY_TRIANGLES;
  for(int k=0; k<isize(indices); k+=3) {
    addtri(make_array(body[indices[k]], body[indices[k+1]], body[indices[k+2]]), kind);
    }
  
  add_texture(sh);
  if(&sh == &shHood || &sh == &shWightCloak || &sh == &shArmor)
    shift_last(-HEAD);
  else
    shift_last(-BODY);
  }

void geometry_information::make_foot_3d(hpcshape& sh) {
  auto foot = get_shape(sh);
  auto leg = get_shape(shHumanLeg);
  auto leg5 = scaleshape(leg, 0.8);

  bshape(sh, PPR::MONSTER_BODY);
  shcenter = get_center(leg);
  add_cone(zc(0), foot, zc(0));
  add_prism(zc(0), foot, zc(0.1), leg);
  add_prism_sync(zc(0.1), leg, zc(0.4), leg5);
  add_cone(zc(0.4), leg5, zc(0.45));
  add_texture(sh);
  // shift_last(-LEG0);
  for(int i=last->s; i<isize(hpc); i++) hpc[i] = lxpush(-0.0125*S) * hpc[i];
  }

void geometry_information::make_head_only() {

  auto addpt = [this] (int d, int u) {
    hpcpush(lzpush(zc(eyepos) + 0.06 * SH * sin(u * degree)) * xspinpush0(d * degree, 0.05 * S * cos(u * degree)));
    };
  
  bshape(shPHeadOnly, shPHeadOnly.prio);
  last->flags |= POLY_TRIANGLES;
  for(int d=0; d<360; d+=(BADMODEL ? 60 : 30)) 
  for(int u=-90; u<=90; u+=(BADMODEL ? 90 : 30)) {
    addpt(d, u);
    addpt(d+30, u);
    addpt(d, u+30);
    addpt(d+30, u+30);
    addpt(d+30, u);
    addpt(d, u+30);
    }
  
  add_texture(shPHeadOnly);
  shift_last(-HEAD - revZ * 0.01 * SH); 
  }


void geometry_information::make_head_3d(hpcshape& sh) {
  auto head = get_shape(sh);
  vector<vector<array<ld, 2> >> pts(2);
  
  for(hyperpoint h: head) {
    array<ld, 2> p;
    p[0] = h[0] / h[LDIM];
    p[1] = h[1] / h[LDIM];
    pts[0].emplace_back(p);
    }
  
  array<ld, 2> zero = make_array<ld>(0,0);
  pts[1].emplace_back(zero);
  head.push_back(tile_center());
  
  bshape(sh, sh.prio);
  
  vector<int> indices = mapbox::earcut<int> (pts);
  
  last->flags |= POLY_TRIANGLES;
  for(int k=0; k<isize(indices); k+=3) {
    addtri(make_array(head[indices[k]], head[indices[k+1]], head[indices[k+2]]), 0);
    }
  
  add_texture(sh);
  shift_last(-HEAD);
  }

void geometry_information::make_paw_3d(hpcshape& sh, hpcshape& legsh) {
  auto foot = get_shape(sh);
  auto leg = get_shape(legsh);

  bshape(sh, PPR::MONSTER_BODY);
  shcenter = get_center(leg);
  add_cone(zc(0), foot, zc(0));
  add_prism(zc(0), foot, zc(0.1), leg);
  add_prism_sync(zc(0.1), leg, zc(0.4), leg);
  add_cone(zc(0.4), leg, zc(0.45));
  add_texture(sh);
  }

void geometry_information::make_abody_3d(hpcshape& sh, ld tail) {
  auto body = get_shape(sh);
  shcenter = get_center(body);

  vector<hyperpoint> notail;
  ld minx = 9;
  for(hyperpoint h: body) minx = min(minx, h[0]);
  for(hyperpoint h: body) if(h[0] >= minx + tail) notail.push_back(h);

  auto body8 = scaleshape(notail, 0.8);  
  
  bshape(sh, PPR::MONSTER_BODY);
  add_prism(zc(0.4), body8, zc(0.45), body);
  add_prism(zc(0.45), body, zc(0.5), notail);
  add_prism_sync(zc(0.6), body8, zc(0.5), notail);
  add_cone(zc(0.4), body8, zc(0.36));
  add_cone(zc(0.6), body8, zc(0.64));
  add_texture(sh);
  if(GDIM == 3 && WDIM == 2) shift_last(-ABODY);
  }

void geometry_information::make_ahead_3d(hpcshape& sh) {
  auto body = get_shape(sh);
  shcenter = get_center(body);
  auto body8 = scaleshape(body, 0.5);
  
  bshape(sh, PPR::MONSTER_BODY);
  add_prism_sync(zc(0.4), body8, zc(0.5), body);
  add_prism_sync(zc(0.6), body8, zc(0.5), body);
  add_cone(zc(0.4), body8, zc(0.36));
  add_cone(zc(0.6), body8, zc(0.64));
  add_texture(sh);
  }

void geometry_information::make_skeletal(hpcshape& sh, ld push) {
  auto body = get_shape(sh);
  shcenter = get_center(body);
  
  bshape(sh, PPR::MONSTER_BODY);
  add_prism_sync(zc(0.48), body, zc(0.5), body);
  add_prism_sync(zc(0.52), body, zc(0.5), body);
  add_cone(zc(0.48), body, zc(0.47));
  add_cone(zc(0.52), body, zc(0.53));
  add_texture(sh);
  shift_last(-push);
  }

hyperpoint yzspin(ld alpha, hyperpoint h) {
  if(embedded_plane) {
    h = cgi.emb->actual_to_logical(h);
    h = cspin(1, 2, alpha) * h;
    h[2] *= cgi.human_height;
    h = cgi.emb->logical_to_actual(h);
    return h;
    }
  if(gproduct) return product::direct_exp(cspin(1, 2, alpha) * product::inverse_exp(h));
  return cspin(1, 2, alpha) * h;
  }

void geometry_information::make_revolution(hpcshape& sh, int mx, ld push) {
  auto body = get_shape(sh);
  bshape(sh, PPR::MONSTER_BODY);
  int step = (BADMODEL ? 60 : (mx == 360 ? 24 : 10));
  for(int i=0; i<isize(body); i++) {
    hyperpoint h0 = body[i];
    hyperpoint h1 = body[(i+1) % isize(body)];
    for(int s=0; s<mx; s+=step) {
      hpcpush(yzspin(s * degree, h0));
      hpcpush(yzspin(s * degree, h1));
      hpcpush(yzspin((s+step) * degree, h0));
      hpcpush(yzspin(s * degree, h1));
      hpcpush(yzspin((s+step) * degree, h0));
      hpcpush(yzspin((s+step) * degree, h1));
      }
    }
  last->flags |= POLY_TRIANGLES;
  add_texture(sh);
  shift_last(-push);
  }

void geometry_information::make_revolution_cut(hpcshape &sh, int each, ld push, ld width) {
  auto body = get_shape(sh);
  body.resize(isize(body) / 2);
  ld fx = cgi.emb->actual_to_logical(body[0])[0];
  ld lx = cgi.emb->actual_to_logical(body.back())[0];
  body.insert(body.begin(), hpxy(fx + (fx-lx) * 1e-3, 0));
  body.push_back(hpxy(lx + (lx-fx) * 1e-3, 0));
  int n = isize(body);
  
  auto gbody = body;
  
  vector<int> nextid(n);
  vector<int> lastid(n);
  vector<bool> stillin(n, true);
  for(int i=0; i<n; i++) nextid[i] = i+1;
  for(int i=0; i<n; i++) lastid[i] = i-1;
  nextid[n-1] = n-1; lastid[0] = 0;

  while(true) {
    int cand = -1;
    ld cv = 0;
    for(int i=1; i<n-1; i++) if(stillin[i]) {
      auto gi = cgi.emb->actual_to_logical(gbody[i]);
      auto gl = cgi.emb->actual_to_logical(gbody[lastid[i]]);
      auto gn = cgi.emb->actual_to_logical(gbody[nextid[i]]);
      if((gi[0] < gl[0] && gi[0] < gn[0]) || (gi[0] > gl[0] && gi[0] > gn[0]) || abs(gi[1]) > width)
      if(abs(gi[1]) > cv)
        cand = i, cv = abs(gi[1]);
      }
    if(cand == -1) break;
    int i = cand;
    lastid[nextid[i]] = lastid[i];
    nextid[lastid[i]] = nextid[i];
    stillin[i] = false;
    }
  
  for(int i=n-1; i>=0; i--) if(!stillin[i] && !stillin[nextid[i]]) nextid[i] = nextid[nextid[i]];
  for(int i=0; i<n; i++) if(!stillin[i] && !stillin[lastid[i]]) lastid[i] = lastid[lastid[i]];

  for(int i=0; i<n; i++) {
    if(!stillin[i]) {
      auto gl = cgi.emb->actual_to_logical(gbody[lastid[i]]);
      auto gn = cgi.emb->actual_to_logical(gbody[nextid[i]]);
      gbody[i] = cgi.emb->logical_to_actual((gl * (i - lastid[i]) + gn * (nextid[i] - i)) / (nextid[i] - lastid[i]));
      }
    }
  
  bshape(sh, PPR::MONSTER_BODY);
  int step = (BADMODEL ? 60 : 10);
  for(int i=0; i<n; i++) {
    for(int s=0; s<360; s+=step) {
      auto& tbody = BADMODEL ? gbody : (s % each ? gbody : body);
      auto& nbody = BADMODEL ? gbody : ((s+step) % each ? gbody : body);
      int i1 = (i+1) % isize(body);
      hyperpoint h0 = tbody[i];
      hyperpoint h1 = tbody[i1];
      hyperpoint hs0 = nbody[i];
      hyperpoint hs1 = nbody[i1];
      hpcpush(yzspin(s * degree, h0));
      hpcpush(yzspin(s * degree, h1));
      hpcpush(yzspin((s+step) * degree, hs0));
      hpcpush(yzspin(s * degree, h1));
      hpcpush(yzspin((s+step) * degree, hs0));
      hpcpush(yzspin((s+step) * degree, hs1));
      }
    }
  last->flags |= POLY_TRIANGLES;
  add_texture(sh);
  shift_last(-push);
  
  if(&sh == &shDogTorso) {
    finishshape();
    shDogStripes = shDogTorso;
    add_texture(shDogStripes);
    auto& utt = models_texture;
    if(utt.tvertices.empty()) return;
    int a = (6 * 360 / step);
    for(int i=0; i<shDogStripes.e - shDogStripes.s; i++)
      if(i % (2 * a) < a)
        utt.tvertices[i + shDogStripes.texture_offset][1] /= 4;
    }
  }

void geometry_information::clone_shape(hpcshape& sh, hpcshape& target) {
  finishshape();
  target = sh;
  target.s = isize(hpc);
  for(int i=sh.s; i<sh.e; i++) hpc.push_back(hpc[i]);
  target.e = isize(hpc);
  }

void geometry_information::animate_bird(hpcshape& orig, hpcshape_animated& animated, ld body) {
  for(int i=0; i<=WINGS; i++) {
    auto& tgt = animated[i];
    clone_shape(orig, tgt);
    ld alpha = cos(M_PI * i / WINGS) * 30 * degree;
    for(int i=tgt.s; i<tgt.e; i++) {
      hyperpoint h = hpc[i];
      h = cgi.emb->actual_to_logical(hpc[i]);
      if(abs(h[1]) > body) {
        ld off = h[1] > 0 ? body : -body;
        h[2] += abs(h[1] - off) * sin(alpha);
        if(embedded_plane) h[2] *= cgi.human_height;
        h[1] = off + (h[1] - off) * cos(alpha);
        h = cgi.emb->logical_to_actual(h);
        h = normalize(h);
        hpc[i] = h;
        }
      }
    }
  // for(int i=0; i<30; i++) shift_shape(animated[i], BIRD);
  // shift_shape(orig, BIRD);
  }

void geometry_information::slimetriangle(hyperpoint a, hyperpoint b, hyperpoint c, ld rad, int lev) {
  dynamicval<int> d(vid.texture_step, 8);
  ld sca = 1;
  if(mhybrid) sca = .5;
  if(cgi.emb->is_euc_in_noniso()) sca *= .3 * geom3::euclid_embed_scale;
  texture_order([&] (ld x, ld y) {
    ld z = 1-x-y;
    ld r = scalefactor * hcrossf7 * (0 + pow(max(x,max(y,z)), .3) * 0.8) * sca;
    hyperpoint h = direct_exp(tangent_length(a*x+b*y+c*z, r));
    hpcpush(h);
    });  
  }

void geometry_information::balltriangle(hyperpoint a, hyperpoint b, hyperpoint c, ld rad, int lev) {
  if(lev == 0) {
    hpcpush(direct_exp(a));
    hpcpush(direct_exp(b));
    hpcpush(direct_exp(c));
    }
  else {
    auto midpoint = [&] (hyperpoint h1, hyperpoint h2) {
      return tangent_length(h1+h2, rad);
      };
    hyperpoint cx = midpoint(a, b);
    hyperpoint ax = midpoint(b, c);
    hyperpoint bx = midpoint(c, a);
    balltriangle(ax, bx, cx, rad, lev-1);
    balltriangle(ax, bx, c , rad, lev-1);
    balltriangle(ax, b , cx, rad, lev-1);
    balltriangle(a , bx, cx, rad, lev-1);
    }
  }

void geometry_information::make_ball(hpcshape& sh, ld rad, int lev) {
  bshape(sh, sh.prio);
  sh.flags |= POLY_TRIANGLES;
  hyperpoint tip = xtangent(rad);
  hyperpoint atip = xtangent(-rad);
  ld z = 63.43 * degree;
  for(int i=0; i<5; i++) {
    hyperpoint a = cspin(1, 2, (72 * i   ) * degree) * spin(z) * tip;
    hyperpoint b = cspin(1, 2, (72 * i-72) * degree) * spin(z) * tip;
    hyperpoint c = cspin(1, 2, (72 * i+36) * degree) * spin(M_PI-z) * tip;
    hyperpoint d = cspin(1, 2, (72 * i-36) * degree) * spin(M_PI-z) * tip;
    balltriangle(tip, a, b, rad, lev);
    balltriangle(a, b, c, rad, lev);
    balltriangle(b, c, d, rad, lev);
    balltriangle(c, d, atip, rad, lev);
    }
  add_texture(sh);
  }

void geometry_information::hpcsquare(hyperpoint h1, hyperpoint h2, hyperpoint h3, hyperpoint h4) {
  last->flags |= POLY_TRIANGLES;
  hpcpush(h1);
  hpcpush(h2);
  hpcpush(h3);
  hpcpush(h2);
  hpcpush(h3);
  hpcpush(h4);
  }

void geometry_information::make_star(hpcshape& sh, ld rad) {
  bshape(sh, sh.prio);
  rad = sinh(rad);
  sh.flags |= POLY_TRIANGLES | POLY_INTENSE;
  int steps = (BADMODEL ? 8 : 64);
  for(int a=0; a<steps; a++) {
    ld z0 = (a-steps/2)*2.0/steps;
    ld z1 = (a-steps/2+1)*2.0/steps;
    ld r0 = sqrt(1 - z0*z0) * rad;
    ld r1 = sqrt(1 - z1*z1) * rad;
    z0 *= rad;
    z1 *= rad;
    int step = BADMODEL ? 60 : 15;
    for(int b=0; b<360; b+=step) {
      ld b0 = b * degree;
      ld b1 = (b+step) * degree;
      hpcsquare(
        hpxy3(r0 * cos(b0), r0 * sin(b0), z0), hpxy3(r0 * cos(b1), r0 * sin(b1), z0), 
        hpxy3(r1 * cos(b0), r1 * sin(b0), z1), hpxy3(r1 * cos(b1), r1 * sin(b1), z1)
        );
      }
    }
  }

void geometry_information::make_euclidean_sky() {
  bshape(cgi.shEuclideanSky, PPR::EUCLIDEAN_SKY);
  for(int x=-20; x<20; x++)
  for(int y=-20; y<20; y++) {
    auto x0 = x * cgi.LOWSKY;
    auto x1 = (x+1) * cgi.LOWSKY;
    auto y0 = y * cgi.LOWSKY;
    auto y1 = (y+1) * cgi.LOWSKY;
    hpcsquare(
      lzpush(cgi.LOWSKY) * hpxy(x0, y0),
      lzpush(cgi.LOWSKY) * hpxy(x0, y1),
      lzpush(cgi.LOWSKY) * hpxy(x1, y0),
      lzpush(cgi.LOWSKY) * hpxy(x1, y1)
      );
    }
  }

/** res[0] and res[1] place H on the plane, while res[2] is the altitude */
hyperpoint psmin(hyperpoint H) {
  if(gproduct) {
    auto d = product_decompose(H);
    d.second[2] = d.first;
    return d.second;
    }
  hyperpoint res;
  res[2] = asin_auto(H[2]);
  ld cs = pow(cos_auto(res[2]), 2);
  ld r = sqrt(cs+H[0]*H[0]+H[1]*H[1]);
  res[0] = H[0] / r;
  res[1] = H[1] / r;
  return res;
  }

void geometry_information::adjust_eye(hpcshape& eye, hpcshape head, ld shift_eye, ld shift_head, int q, ld zoom) {
  hyperpoint center = Hypc;
  int c = 0;
  for(int i=eye.s; i<eye.e; i++) if(q == 1 || hpc[i][1] > 0) center += hpc[i], c++;
  center /= c;
  center = cgi.emb->normalize_flat(center);
  // center /= (eye.e - eye.s);
  ld rad = 0;
  for(int i=eye.s; i<eye.e; i++) if(q == 1 || hpc[i][1] > 0) rad += hdist(center, hpc[i]);
  rad /= (eye.e - eye.s);
  
  hyperpoint pscenter = psmin(center);
  
  ld pos = 0;
  
  vector<hyperpoint> pss;
  
  for(int i=head.s; i<head.e; i++) pss.push_back(psmin(lzpush(shift_head - cgi.emb->center_z()) * hpc[i]));
  
  ld zmid = 0;
  for(hyperpoint& h: pss) zmid += h[2];
  zmid /= isize(pss);
  
  ld mindist = 1e9;
  for(int i=0; i<isize(pss); i+=3) if(pss[i][2] < zmid || (WDIM == 3 && !gproduct)) {
    ld d = sqhypot_d(2, pss[i]-pscenter) + sqhypot_d(2, pss[i+1]-pscenter) + sqhypot_d(2, pss[i+2]-pscenter);
    if(d < mindist) mindist = d, pos = min(min(pss[i][2], pss[i+1][2]), pss[i+2][2]);
    }
  
  if(&eye == &shSkullEyes) cgi.eyelevel_human = pos = zc(eyepos) - 0.06 * SH * 0.05;
  if(&eye == &shWolf1) cgi.eyelevel_dog = pos;
  if(&eye == &shFamiliarEye) cgi.eyelevel_familiar = pos;
  
  make_ball(eye, rad, 0);
  transmatrix T = lzpush(-shift_eye) * rgpushxto0(center) * lzpush(pos);
  for(int i=eye.s; i<isize(hpc); i++) hpc[i] = T * hpc[i];
  int s = isize(hpc);
  if(&eye == &shSkullEyes) 
    for(int i=eye.s; i<s; i++) hpc[i] = lxpush(0.07 * scalefactor) * hpc[i];
  if(q == 2)
    for(int i=eye.s; i<s; i++) {
      hpcpush(MirrorY * hpc[i]);
      auto& utt = models_texture;
      if(!utt.tvertices.empty())
        utt.tvertices.push_back(utt.tvertices[i - eye.s + eye.texture_offset]);
      }

  finishshape();
  // eye.prio = PPR::SUPERLINE;
  }

void geometry_information::shift_last_straight(ld z) {
  for(int i=last->s; i<isize(hpc); i++) hpc[i] = lzpush(z) * hpc[i];
  }

EX void queueball(const shiftmatrix& V, ld rad, color_t col, eItem what) {
  if(what == itOrbSpeed) {
    shiftmatrix V1 = V * cspin90(1, 2);
    ld tt = ptick(100);
    for(int t=0; t<5; t++) {
      for(int a=-50; a<50; a++)
        curvepoint(cspin(0, 2, a * M_PI/100.) * cspin(0, 1, t * 72._deg + tt + a*TAU/50.) * xpush0(rad));
      queuecurve(V1, col, 0, PPR::LINE);
      }
    return;
    }
  ld z = 63.43 * degree;
  shiftmatrix V1 = V * cspin90(0, 2);
  if(what == itOrbShield) V1 = V * cspin(0, 1, ptick(500));
  if(what == itOrbFlash) V1 = V * cspin(0, 1, ptick(1500));
  if(what == itOrbShield) V1 = V * cspin(1, 2, ptick(1000));
  if(what == itOrbFlash) V1 = V * cspin(1, 2, ptick(750));
  if(what == itDiamond) V1 = V * cspin(1, 2, ptick(200));
  if(what == itRuby) V1 = V * cspin(1, 2, ptick(300));
  auto line = [&] (transmatrix A, transmatrix B) {
    hyperpoint h0 = A * xpush0(1);
    hyperpoint h1 = B * xpush0(1);
    for(int i=0; i<=8; i++)
      curvepoint(rspintox(normalize(h0*(8-i) + h1*i)) * xpush0(rad));
    queuecurve(V1, col, 0, PPR::LINE);
    };
  for(int i=0; i<5; i++) {
    auto a = cspin(1, 2, (72 * i   ) * degree) * spin(z);
    auto b = cspin(1, 2, (72 * i-72) * degree) * spin(z);
    auto c = cspin(1, 2, (72 * i+36) * degree) * spin(M_PI-z);
    auto d = cspin(1, 2, (72 * i-36) * degree) * spin(M_PI-z);
    line(Id, a);
    line(a, b);
    line(a, c);
    line(a, d);
    line(d, c);
    line(c, spin180());
    }
  }

void geometry_information::make_shadow(hpcshape& sh) {
  sh.shs = isize(hpc);
  for(int i=sh.s; i < sh.e; i++) hpcpush(orthogonal_move(hpc[i], FLOOR - human_height / 100));
  sh.she = isize(hpc);
  }

void geometry_information::make_3d_models() {
  if(GDIM == 2 || noGUI) return;
  eyepos = WDIM == 2 ? 0.875 : 0.925;
  DEBBI(DF_POLY, ("make_3d_models"));
  shcenter = tile_center();

#if CAP_GL  
  if(floor_textures) {
    auto& utt = models_texture;
    utt.tvertices.clear();
    utt.texture_id = floor_textures->renderedTexture;
    }
#endif
  
  if(WDIM == 2 || euclid) {
    DEBB(DF_POLY, ("shadows"));
    for(hpcshape* sh: {&shBatWings, &shBugBody, &shBullBody, &shButterflyWing, &shCatBody, &shDogBody, &shDogTorso,
      &shEagle, &shFemaleBody, &shFlailMissile, &shGadflyWing, &shGargoyleWings, &shHawk, &shJiangShi, &shKnife,
      &shPBody, &shPHead, &shRaiderBody, &shReptileBody, &shSkeletonBody, &shTongue, &shTrapArrow, &shTrylobite,
      &shWaterElemental, &shWolfBody, &shYeti, &shWormHead, &shWormHead, &shDragonHead, &shDragonSegment, &shDragonTail,
      &shTentacleX, &shTentHead, &shILeaf[0], &shILeaf[1], &shWormSegment, &shSmallWormSegment,
      &shWormTail, &shSmallWormTail, &shFrogBody })
      make_shadow(*sh);
    
    for(int i=0; i<8; i++) make_shadow(shAsteroid[i]);
    }
  
  DEBB(DF_POLY, ("humanoids"));
  make_humanoid_3d(shPBody);
  make_humanoid_3d(shYeti);
  make_humanoid_3d(shFemaleBody);
  make_humanoid_3d(shRaiderBody);
  make_humanoid_3d(shSkeletonBody);
  make_humanoid_3d(shFatBody);
  make_humanoid_3d(shWaterElemental);
  make_humanoid_3d(shJiangShi);
  
  // shFatBody = shPBody;
  // shFemaleBody = shPBody;
  // shRaiderBody = shPBody;
  // shJiangShi = shPBody;

  DEBB(DF_POLY, ("heads"));
  make_head_3d(shFemaleHair);
  make_head_3d(shPHead);
  make_head_3d(shTurban1);
  make_head_3d(shTurban2);
  make_head_3d(shAztecHead);
  make_head_3d(shAztecCap);
  make_head_3d(shVikingHelmet);
  make_head_3d(shRaiderHelmet);
  make_head_3d(shWestHat1);
  make_head_3d(shWestHat2);
  make_head_3d(shWitchHair);
  make_head_3d(shBeautyHair);
  make_head_3d(shFlowerHair);
  make_head_3d(shGolemhead);
  make_head_3d(shPirateHood);
  make_head_3d(shEyepatch);
  make_head_3d(shSkull);
  make_head_3d(shDemon);
  make_head_3d(shGoatHead);
  make_head_3d(shJiangShiCap1);
  make_head_3d(shJiangShiCap2);
  make_head_3d(shTerraHead);
  
  DEBB(DF_POLY, ("armors"));
  make_armor_3d(shKnightArmor);
  make_armor_3d(shKnightCloak, 2);
  make_armor_3d(shPrinceDress);
  make_armor_3d(shPrincessDress, 2);
  make_armor_3d(shTerraArmor1);
  make_armor_3d(shTerraArmor2);
  make_armor_3d(shTerraArmor3); 
  make_armor_3d(shSuspenders); 
  make_armor_3d(shJiangShiDress); 
  make_armor_3d(shFemaleDress); 
  make_armor_3d(shWightCloak, 2); 
  make_armor_3d(shRaiderArmor); 
  make_armor_3d(shRaiderShirt); 
  make_armor_3d(shArmor); 
  make_armor_3d(shRatCape2, 2); 
  
  make_armor_3d(shHood, 2);
  
  DEBB(DF_POLY, ("feet and paws"));
  make_foot_3d(shHumanFoot);
  make_foot_3d(shYetiFoot);
  make_skeletal(shSkeletalFoot, WDIM == 2 ? zc(0.5) + human_height/40 - FLOOR : 0);
  
  hyperpoint front_leg = Hypc;
  hyperpoint rear_leg = Hypc;
  for(int i=shDogFrontPaw.s; i<shDogFrontPaw.e; i++) front_leg += hpc[i];
  for(int i=shDogRearPaw.s; i<shDogRearPaw.e; i++) rear_leg += hpc[i];
  front_leg = normalize(front_leg);
  rear_leg = normalize(rear_leg);
  front_leg_move = lzpush(zc(0.4)) * rgpushxto0(front_leg);
  front_leg_move_inverse = inverse(front_leg_move);
  rear_leg_move = lzpush(zc(0.4)) * rgpushxto0(rear_leg);
  rear_leg_move_inverse = inverse(rear_leg_move);
  leg_length = zc(0.4) - zc(0);
  
  make_paw_3d(shWolfFrontPaw, shWolfFrontLeg);
  make_paw_3d(shWolfRearPaw, shWolfRearLeg);
  make_paw_3d(shDogFrontPaw, shDogFrontLeg);
  make_paw_3d(shDogRearPaw, shDogRearLeg);  
  
  DEBB(DF_POLY, ("revolution"));
  // make_abody_3d(shWolfBody, 0.01);
  // make_ahead_3d(shWolfHead);
  // make_ahead_3d(shFamiliarHead);
  ld g = WDIM == 2 ? ABODY - zc(0.4) : 0;
  
  make_revolution_cut(shWolfBody, 30, g, 0.01*S);
  make_revolution_cut(shWolfHead, 180, AHEAD - ABODY +g);
  make_revolution_cut(shRatHead, 180, AHEAD - ABODY +g, 0.04*scalefactor);
  make_revolution_cut(shRatCape1, 180, AHEAD - ABODY +g);
  make_revolution_cut(shFamiliarHead, 30, AHEAD - ABODY +g);

  // make_abody_3d(shDogTorso, 0.01);
  make_revolution_cut(shDogTorso, 30, +g);
  make_revolution_cut(shDogHead, 180, AHEAD - ABODY +g);
  // make_ahead_3d(shDogHead);

  // make_abody_3d(shCatBody, 0.05);
  // make_ahead_3d(shCatHead);
  make_revolution_cut(shCatBody, 30, +g);
  make_revolution_cut(shCatHead, 180, AHEAD - ABODY +g, 0.055 * scalefactor);

  make_paw_3d(shReptileFrontFoot, shReptileFrontLeg);
  make_paw_3d(shReptileRearFoot, shReptileRearLeg);  
  make_abody_3d(shReptileBody, -1);
  // make_ahead_3d(shReptileHead);
  make_revolution_cut(shReptileHead, 180, AHEAD - ABODY+g);

  make_paw_3d(shBullFrontHoof, shBullFrontHoof);
  make_paw_3d(shBullRearHoof, shBullRearHoof);
  // make_abody_3d(shBullBody, 0.05);
  // make_ahead_3d(shBullHead);
  // make_ahead_3d(shBullHorn);
  make_revolution_cut(shBullBody, 180, +g);
  make_revolution_cut(shBullHead, 60, AHEAD - ABODY +g);
  shift_shape(shBullHorn, -g-(AHEAD - ABODY));
  // make_revolution_cut(shBullHorn, 180, AHEAD - ABODY);
  
  make_paw_3d(shFrogFrontFoot, shFrogFrontLeg);
  make_paw_3d(shFrogRearFoot, shFrogRearLeg);
  make_paw_3d(shFrogJumpFoot, shFrogJumpLeg);
  
  make_paw_3d(shTrylobiteFrontClaw, shTrylobiteFrontLeg);
  make_paw_3d(shTrylobiteRearClaw, shTrylobiteRearLeg);
  make_abody_3d(shTrylobiteBody, 0);
  // make_ahead_3d(shTrylobiteHead);
  make_revolution_cut(shTrylobiteHead, 180, AHEAD - ABODY +g);
  
  make_revolution_cut(shShark, 180, WDIM == 2 ? -FLOOR : 0);
  make_revolution_cut(shPikeBody, 180, WDIM == 2 ? -FLOOR : 0);

  make_revolution_cut(shGhost, 60, GHOST + g);

  make_revolution_cut(shEagle, 180, 0, 0.05*S);
  make_revolution_cut(shHawk, 180, 0, 0.05*S);

  make_revolution_cut(shTinyBird, 180, 0, 0.025 * S);
  make_revolution_cut(shTinyShark, 90);
  make_revolution_cut(shMiniGhost, 60);

  make_revolution_cut(shGargoyleWings, 180, 0, 0.05*S);
  make_revolution_cut(shGargoyleBody, 180, 0, 0.05*S);
  make_revolution_cut(shGadflyWing, 180, 0, 0.05*S);
  make_revolution_cut(shBatWings, 180, 0, 0.05*S);
  make_revolution_cut(shBatBody, 180, 0, 0.05*S);
  
  make_revolution_cut(shMouse, 180, -FLOOR);
  shift_shape(shMouseLegs, FLOOR - human_height / 200);

  make_revolution_cut(shJelly, 60);
  make_revolution(shFoxTail1);
  make_revolution(shFoxTail2);
  make_revolution(shGadflyBody, 180, 0);
  for(int i=0; i<8; i++)
    make_revolution(shAsteroid[i], 360);
  
  make_revolution_cut(shBugLeg, 60);

  make_revolution(shBugArmor, 180, ABODY);
  make_revolution_cut(shBugAntenna, 90, ABODY);
  
  make_revolution(shFrogBody, 180, WDIM == 2 ? g : ABODY);
  
  make_revolution_cut(shButterflyBody, 180, 0);
  make_revolution_cut(shButterflyWing, 180, 0, 0.05*S);
  finishshape();
  
  DEBB(DF_POLY, ("animatebirds"));
  animate_bird(shEagle, shAnimatedEagle, 0.05*S);
  animate_bird(shTinyBird, shAnimatedTinyEagle, 0.05*S/2);

  animate_bird(shButterflyWing, shAnimatedButterfly, 0);
  animate_bird(shGadflyWing, shAnimatedGadfly, 0);
  animate_bird(shHawk, shAnimatedHawk, 0.05*S);
  animate_bird(shGargoyleWings, shAnimatedGargoyle, 0.05*S);
  animate_bird(shGargoyleBody, shAnimatedGargoyle2, 0.05*S);
  animate_bird(shBatWings, shAnimatedBat, 0.05*S);
  animate_bird(shBatBody, shAnimatedBat2, 0.05*S);

  DEBB(DF_POLY, ("disablers"));

  disable(shWolfRearLeg);
  disable(shWolfFrontLeg);
  disable(shDogRearLeg);
  disable(shDogFrontLeg);
  disable(shReptileFrontLeg);
  disable(shReptileRearLeg);
  disable(shTrylobiteFrontLeg);
  disable(shTrylobiteRearLeg);
  disable(shPFace);
  disable(shJiangShi);
  
  disable(shFrogFrontLeg);
  disable(shFrogRearLeg);
  disable(shFrogRearLeg2);
  disable(shFrogJumpLeg);
  
  make_revolution_cut(shDragonSegment, 60, g);
  make_revolution_cut(shDragonHead, 60, g);
  make_revolution_cut(shDragonTail, 60, g);
  make_revolution_cut(shWormSegment, 60, g);
  make_revolution_cut(shSmallWormSegment, 60, g);
  make_revolution_cut(shWormHead, 60, g);
  make_revolution_cut(shWormTail, 60, g);
  make_revolution_cut(shSmallWormTail, 60, g);
  make_revolution_cut(shTentHead, 60, g);
  make_revolution_cut(shKrakenHead, 60, -FLOOR);
  make_revolution_cut(shSeaTentacle, 60, -FLOOR);
  make_revolution_cut(shDragonLegs, 60, g);
  make_revolution_cut(shDragonWings, 60, g);
  disable(shDragonNostril);

  make_head_only();
  
  DEBB(DF_POLY, ("balls"));
  make_ball(shDisk, orbsize*.2, 2);
  make_ball(shHeptaMarker, zhexf*.2, 1);
  make_ball(shSnowball, zhexf*.1, 1);
  make_ball(shSkyboxSun, 8 * zhexf, 2);
  if(euclid) make_euclidean_sky();
  make_star(shSun, vid.sun_size * zhexf);
  make_star(shNightStar, vid.star_size * zhexf);
  
  if(WDIM == 2) {
    for(int i=0; i<3; i++) {
      clone_shape(shHalfFloor[i], shHalfFloor[i+3]);
      shift_shape_orthogonally(shHalfFloor[i], FLOOR - human_height * .007);
      shift_shape_orthogonally(shHalfFloor[i+3], WALL + human_height * .007);
      }
    }

  shift_shape(shBoatOuter, FLOOR);
  shift_shape(shBoatInner, (FLOOR+LAKE)/2);
  
  for(int i=0; i<14; i++)
    shift_shape(shTriheptaSpecial[i], FLOOR);

  shift_shape_orthogonally(shBigCarpet1, FLOOR - human_height * 1/40);
  shift_shape_orthogonally(shBigCarpet2, FLOOR - human_height * 2/40);
  shift_shape_orthogonally(shBigCarpet3, FLOOR - human_height * 3/40);
  for(int a=0; a<5; a++) for(int b=0; b<4; b++)
    shift_shape(shReptile[a][b], FLOOR - human_height * min(b, 2) / 40);
    
  shift_shape(shMineMark[0], FLOOR - human_height * 1/40);
  shift_shape(shMineMark[1], FLOOR - human_height * 1/40);

  for(int a=0; a<5; a++) shift_shape(shZebra[a], FLOOR);
  
  auto tortz = [] (int t) {
    if(t == 0) return 2;
    else if(t < 8) return 3;
    else if(t == 13) return 2;
    else return 1;
    };
  
  for(int t=0; t<13; t++) for(int u=0; u<4; u++)
    shift_shape(shTortoise[t][u], FLOOR - human_height * tortz(t) / 120);

  make_revolution_cut(shStatue, 60);
  
  shift_shape(shThorns, FLOOR - human_height * 1/40);
  clone_shape(shRose, shRoseItem);
  shift_shape(shRose, FLOOR - human_height * 1/20);

  DEBB(DF_POLY, ("slime"));
  bshape(shSlime, PPR::MONSTER_BODY);
  hyperpoint tip = xtangent(1);
  hyperpoint atip = xtangent(-1);
  ld z = 63.43 * degree;
  for(int i=0; i<5; i++) {
    auto a = cspin(1, 2, (72 * i   ) * degree) * cspin(0, 1, z) * xtangent(1);
    auto b = cspin(1, 2, (72 * i-72) * degree) * cspin(0, 1, z) * xtangent(1);
    auto c = cspin(1, 2, (72 * i+36) * degree) * cspin(0, 1, M_PI-z) * xtangent(1);
    auto d = cspin(1, 2, (72 * i-36) * degree) * cspin(0, 1, M_PI-z) * xtangent(1);
    slimetriangle(tip, a, b, 1, 0);
    slimetriangle(a, b, c, 1, 0);
    slimetriangle(b, c, d, 1, 0);
    slimetriangle(c, d, atip, 1, 0);    
    }
  last->flags |= POLY_TRIANGLES;
  add_texture(*last);
  if(WDIM == 2) shift_last_straight(FLOOR + cgi.emb->center_z());
  finishshape();
  shJelly = shSlime;
  
  shift_shape(shMagicSword, ABODY);
  shift_shape(shMagicShovel, ABODY);
  
  DEBB(DF_POLY, ("eyes"));
  adjust_eye(shSlimeEyes, shSlime, FLATEYE, 0, 2, 2);
  adjust_eye(shGhostEyes, shGhost, GHOST, GHOST, 2, WDIM == 2 ? 2 : 4);
  adjust_eye(shMiniEyes, shMiniGhost, GHOST, GHOST, 2, 2);
  adjust_eye(shWormEyes, shWormHead, 0, 0, 2, 4);
  adjust_eye(shDragonEyes, shDragonHead, 0, 0, 2, 4);
  
  adjust_eye(shKrakenEye, shKrakenHead, 0, 0, 1);
  adjust_eye(shKrakenEye2, shKrakenEye, 0, 0, 1, 2);
  
  shRatEye1 = shWolf1;
  shRatEye2 = shWolf2;
  shRatEye3 = shWolf3;

  adjust_eye(shWolf1, shDogHead, AHEAD, AHEAD, 1); 
  adjust_eye(shWolf2, shDogHead, AHEAD, AHEAD, 1); 
  for(int i=0; i<shWolf1.e-shWolf1.s; i++)
    hpc[shWolf2.s+i] = MirrorY * hpc[shWolf1.s+i];
  adjust_eye(shWolf3, shDogHead, AHEAD, AHEAD, 1);
  adjust_eye(shFamiliarEye, shWolfHead, AHEAD, AHEAD, 1);

  adjust_eye(shRatEye1, shRatHead, AHEAD, AHEAD, 1); 
  adjust_eye(shRatEye2, shRatHead, AHEAD, AHEAD, 1); 
  for(int i=shRatEye3.s; i<shRatEye3.e; i++) hpc[i] = xpush(-scalefactor * 0.02) * hpc[i];
  adjust_eye(shRatEye3, shRatHead, AHEAD, AHEAD, 1);
  
  adjust_eye(shWolfEyes, shWolfHead, AHEAD, AHEAD, 1);
  
  adjust_eye(shPikeEye, shPikeBody, 0, 0, 1);
  adjust_eye(shFrogEye, shFrogBody, 0, 0, 1);

  adjust_eye(shReptileEye, shReptileHead, AHEAD, AHEAD, 1);
  adjust_eye(shGadflyEye, shGadflyBody, 0, 0, 1);
  
  adjust_eye(shSkullEyes, shPHeadOnly, HEAD1, HEAD, 2, 2);
  shSkullEyes.tinf = NULL;

  adjust_eye(shMouseEyes, shMouse, FLOOR, FLOOR, 2, 1);

  shift_shape(shRatTail, zc(0.5) - LEG);
  for(int i=shRatTail.s; i<shRatTail.e; i++) hpc[i] = xpush(-scalefactor * 0.1) * hpc[i];

  shift_shape(shSemiFloorShadow, FLOOR - human_height / 100);
  shift_shape(shSemiFloor[0], WALL);
  
  bshape(shPalaceGate, PPR::WALL);
  for(int i=0; i<4; i++) {
    ld x = -0.219482 + (0.135153 + 0.219482) * (i+.5) / 4;
    for(int j=0; j<12; j++) {
      hyperpoint left = xpush(x) * xspinpush0(j * 30 * degree, 0.02);
      hyperpoint right = xpush(x) * xspinpush0((j+1) * 30 * degree, 0.02);
      hpcpush(orthogonal_move(left, FLOOR));
      hpcpush(orthogonal_move(right, FLOOR));
      hpcpush(orthogonal_move(left, WALL));
      hpcpush(orthogonal_move(right, FLOOR));
      hpcpush(orthogonal_move(left, WALL));
      hpcpush(orthogonal_move(right, WALL));
      }
    }
  shPalaceGate.flags |= POLY_TRIANGLES;
  
  for(int i=0; i<3; i++) {
  
    array<hpcshape*,3> sh = make_array(&shBigCarpet1, &shBigCarpet2, &shBigCarpet3);

    bshape(*sh[i], PPR::GFLOORa);  
    for(int t=0; t<S7; t++) {
      dynamicval<int> dv(vid.texture_step, 16);
      texture_order([&] (ld x, ld y) {
        ld z = 1-x-y;
        ld rad = 2.1 - i * 0.2;
        hyperpoint hx = ddi(t*12, -zhexf*rad) * tile_center();
        hyperpoint hy = ddi(t*12+12, -zhexf*rad) * tile_center();
        hyperpoint hz = tile_center();
        hyperpoint h = hx * x + hy * y + hz * z;
        h = mid(h, h);
        h = orthogonal_move(h, FLOOR - human_height * (i+2.5) / 100 * (x+y+1)/2);
        hpcpush(h);
        });  
      }
      
    sh[i]->flags |= POLY_TRIANGLES;
    }

  finishshape();
  }

hpcshape& geometry_information::gen_pipe(hpcshape& pipe, ePipeEnd endtype, ld ratio, const hr::function<hyperpoint(ld,ld,ld)>& f) {
  cgi.bshape(pipe, PPR::WALL);

  #if CAP_GL
  auto& utt = cgi.models_texture;
  if(floor_textures) {
    pipe.tinf = &utt;
    pipe.texture_offset = isize(utt.tvertices);
    }
  #endif

  const int MAX_X = 32;
  const int MAX_R = 20;
  auto at = [&] (ld i, ld a, ld z = 1, ld s = 1) {
    a += 0.5;
    ld alpha = TAU * a / MAX_R;
    cgi.hpcpush(f(i / MAX_X, alpha, z));
    #if CAP_GL
    if(floor_textures) utt.tvertices.push_back(glhr::makevertex(0, true ? 0.549 - s * 0.45 * sin(alpha) : 0.999, 0));
    #endif
    };
  for(int i=0; i<MAX_X; i++) {
    for(int a=0; a<MAX_R; a++) {
      at(i, a, 1);
      at(i, a+1, 1);
      at(i+1, a, 1);
      at(i+1, a+1, 1);
      at(i+1, a, 1);
      at(i, a+1, 1);
      }
    }

  if(endtype == ePipeEnd::sharp) for(int a=0; a<MAX_R; a++) for(int x: {0, MAX_X}) {
    at(x, a, 1, 0);
    at(x, a+1, 1, 0);
    at(x, 0, 0, 0);
    }

  if(endtype == ePipeEnd::ball) for(int a=0; a<MAX_R; a++) for(int x=-MAX_R; x<MAX_R; x++) {
    ld xb = x < 0 ? 0 : MAX_X;
    ld mul = MAX_X * ratio * .9; // .9 to prevent Z-fighting
    ld x0 = xb + mul * sin(x * 90._deg / MAX_R);
    ld x1 = xb + mul * sin((x+1) * 90._deg / MAX_R);
    ld z0 = cos(x * 90._deg / MAX_R);
    ld z1 = cos((x+1) * 90._deg / MAX_R);
    at(x0, a, z0, z0);
    at(x0, a+1, z0, z0);
    at(x1, a, z1, z1);
    at(x1, a+1, z1, z1);
    at(x1, a, z1, z1);
    at(x0, a+1, z0, z0);
    }

  last->flags |= POLY_TRIANGLES | POLY_PRINTABLE;
  finishshape();
  extra_vertices();
  return pipe;
  }

hpcshape& geometry_information::get_pipe_noniso(hyperpoint target, ld width, ePipeEnd endtype) {
  int id = bucketer(target)  + int(157003 * log(width+.001));
  if(cgi.shPipe.count(id)) return cgi.shPipe[id];
  hpcshape& pipe = cgi.shPipe[id];

  hyperpoint lmax = sol ? inverse_exp_newton(target, 10) : inverse_exp(shiftless(target));
  transmatrix lT;
  ld length;
  if(1) {
    dynamicval<eGeometry> g(geometry, gCubeTiling);
    length = hdist0(lmax);
    lT = rspintox(lmax);
    }

  return gen_pipe(pipe, endtype, width/length, [&] (ld i, ld alpha, ld z) {
    hyperpoint p;
    if(1) {
      dynamicval<eGeometry> g(geometry, gCubeTiling);
      p = xpush(i * length) * cspin(1, 2, alpha) * ypush0(width*z);
      p = lT * p;
      }
    return direct_exp(p);
    });
  }

hpcshape& geometry_information::get_pipe_iso(ld length, ld width, ePipeEnd endtype) {
  int id = int(length * 172 + .5) + int(157003 * log(width+.001));
  bool pers = in_perspective();
  if(!pers) id ^= 0x4126891;
  if(shPipe.count(id)) return shPipe[id];
  hpcshape& pipe = shPipe[id];
  println(hlog, "generating pipe of length ", length, " and width ", width);

  return gen_pipe(pipe, endtype, width/length, [&] (ld i, ld alpha, ld z) { return xpush(i * length) * cspin(1, 2, alpha) * ypush0(width*z); });
  }

#undef S
#undef SH
#undef revZ
#endif

}
