// Hyperbolic Rogue -- Barriers
// Copyright (C) 2011-2019 Zeno Rogue, see 'hyper.cpp' for details

/** \file barriers.cpp 
 *  \brief This file implements routines related to barriers (Great Walls and similar).
 */

#include "hyper.h"
namespace hr {

EX bool checkBarriersFront(cellwalker bb, int q IS(5), bool cross IS(false)) {

  if(!ctof(bb.at)) 
    return false;

  if(bb.at->mpdist < BARLEV) return false;
  if(bb.at->mpdist == BUGLEV) return false;
  if(bb.at->bardir != NODIR) return false;
  if(bb.spin == (PURE ? 3 : 0)) {q--; if(!q) return true; }

  if(!cross) for(int i=0; i<7; i++) {
    cellwalker bb2 = bb + i + wstep;
    if(bb2.at->bardir != NODIR) return false; 
    if(!PURE) { 
      bb2 = bb2 + 4 + wstep;
      if(bb2.at->bardir != NODIR) return false;
      }
    }

  bb += wstep;
  if(!PURE) { bb = bb + 3 + wstep + 3 + wstep; }
  return checkBarriersBack(bb, q);
  }

/** return true if the cell c is not allowed to generate barriers because of other large things already existing nearby. */
EX bool hasbardir(cell *c) {
  return c->bardir != NODIR && c->bardir != NOBARRIERS && c->bardir != NOBARRIERS2;
  }

EX void preventbarriers(cell *c) {
  if(mhybrid) c = hybrid::get_where(c).first;
  if(c && c->bardir == NODIR) c->bardir = NOBARRIERS;
  }

EX bool checkBarriersBack(cellwalker bb, int q IS(5), bool cross IS(false)) {
  // printf("back, %p, s%d\n", hr::voidp(bb.at), bb.spin);

  // if(mark) { printf("mpdist = %d [%d] bardir = %d spin=%d q=%d cross=%d\n", bb.at->mpdist, BARLEV, bb.at->bardir, bb.spin, q, cross); }
  
  if(bb.at->mpdist < BARLEV) return false;
  if(bb.at->mpdist == BUGLEV) return false;
  if(bb.at->bardir != NODIR) return false;

  // if(bb.spin == 0 && bb.at->mpdist == INFD) return true;
  
  if(!cross) for(int i=0; i<7; i++) {
    cellwalker bb2 = bb + i + wstep;
    if(bb2.at->bardir != NODIR) return false;
    if(!PURE) {
      bb2 = bb2 + 4 + wstep;
      if(bb2.at->bardir != NODIR) return false;
      }
    }

  bb = bb + 3 + wstep + (PURE ? 5 : 4) + wstep + 3;
  return checkBarriersFront(bb, q);
  }

/** warp coasts use a different algorithm for nowall barriers when has_nice_dual() is on. Check whether we should use this different algorithm when the lands are l1 and l2 */
EX bool warped_version(eLand l1, eLand l2) {
  return (has_nice_dual() && (l1 == laWarpCoast || l1 == laWarpSea || l2 == laWarpSea || l2 == laWarpCoast));
  }

EX int get_valence(cellwalker bb, int dir, bool& ok) {
  if(arb::in() && arb::current_or_slided().have_valence) {
    auto& sh = arb::current_or_slided().shapes[arb::id_of(bb.at->master)];
    if(bb.mirrored) dir = -dir;
    if(dir == 1)
      return sh.vertex_valence[bb.spin];
    else
      return sh.vertex_valence[gmod(bb.spin - 1, bb.at->type)];
    }
  int steps = 0;
  cellwalker bb1 = bb;
  while(bb1 != bb || !steps) {
    bb1 += dir;
    bb1 += wstep;
    steps++;
    }
  return steps;
  }

EX void set_and_wall(cell *c, eLand l) {
  setland(c, l);
  if(c->bardir == NODIR) {
    c->barleft = NOWALLSEP_USED;
    c->bardir = NOBARRIERS;
    }
  }

EX void surround_by(bool setit, cellwalker bb, int dir, int a, int b, eLand which, bool swapped, bool& ok) {
  for(int i=0; i<a; i++) {
    bb += dir;
    bb += wstep;
    }
  for(int i=a; i<b; i++) {
    if(setit) set_and_wall(bb.at, which);
    else {
      if(bb.at->bardir != NODIR) ok = false;
      if(swapped && bb.at->mpdist < BARLEV) ok = false;
      }
    bb += dir;
    bb += wstep;
    }
  }

EX void spin_around_by(cellwalker& bb, int dir, int q) {
  for(int i=0; i<q; i++) {
    bb += dir;
    bb += wstep;
    }
  }

EX bool on_wall(eLand ws) {
  return among(ws, NOWALLSEP_WALL_CPOS, NOWALLSEP_WALL_CNEG, NOWALLSEP_WALL_EPOS, NOWALLSEP_WALL_ENEG);
  }

string wsname(eLand ws) {
  if(ws == NOWALLSEP) return "NO";
  if(ws == NOWALLSEP_SWAP) return "SWAP";
  if(ws == NOWALLSEP_USED) return "USED";
  if(ws == NOWALLSEP_WALL) return "WALL";
  if(ws == NOWALLSEP_WALL_CPOS) return "CPOS";
  if(ws == NOWALLSEP_WALL_CNEG) return "CNEG";
  if(ws == NOWALLSEP_WALL_EPOS) return "EPOS";
  if(ws == NOWALLSEP_WALL_ENEG) return "ENEG";
  return dnameof(ws);
  }

EX bool general_barrier_advance(cellwalker& bb, int& dir, eLand& l1, eLand& l2, eLand& ws, bool setit) {
  bool ok = true;
  if(ws == NOWALLSEP_WALL) {
  

    /*
    if(setit) bb.at->monst = moBug0;
    if(setit) bb.cpeek()->monst = moBug2;
    */

    int steps = get_valence(bb, dir, ok);
    if(steps % 2 == 0) goto again;
    if(steps >= OINF) return ok;

    int s = (steps - 1) / 2;
    surround_by(setit, bb, dir, 1, s, l1, false, ok);
    surround_by(setit, bb, dir, s+1, steps-1, l2, true, ok);
    spin_around_by(bb, dir, s);

    ws = dir > 0 ? NOWALLSEP_WALL_CPOS : NOWALLSEP_WALL_CNEG;
    // goto tile;
    return ok;
    }

  if(on_wall(ws)) {

    bool at_corner = among(ws, NOWALLSEP_WALL_CPOS, NOWALLSEP_WALL_CNEG);

    cell *current = bb.at;

    // if at_corner: bb is facing the tile 1 before the first inside
    int t = bb.at->type;

    int q = at_corner ? t/2 : (t-1) / 2;
    
    if(1) {
      auto bb1 = bb;
      if(at_corner) bb1 += dir;
      bb1 -= dir;
      for(int i=1; i<q; i++) {
        int d = get_valence(bb1, -dir, ok);
        if(d >= OINF) return ok;
        surround_by(setit, bb1, -dir, 1, d, l1, false, ok);
        bb1-=dir;
        }
      }
    
    bb += dir;
    for(int i=1; i<q; i++) {
      int d = get_valence(bb, dir, ok);
      if(d >= OINF) return ok;
      surround_by(setit, bb, dir, 1, d, l2, true, ok);
      bb+=dir;
      }

    // bb is now facing the last neighbor inside 

    if(t % 2 == (at_corner ? 1 : 0)) {
      if(setit) setbarrier(current, l1, l2, at_corner);
      int d = get_valence(bb, dir, ok);
      if(d >= OINF) return ok;
      surround_by(setit, bb, dir, 2, d, l2, true, ok);

      bb += dir;
      bb += wstep;

      d = get_valence(bb, -dir, ok);
      if(d >= OINF) return ok;
      surround_by(setit, bb, -dir, 2, d-1, l1, false, ok);

      ws = dir > 0 ? NOWALLSEP_WALL_EPOS : NOWALLSEP_WALL_ENEG;
      return ok;
      }
    
    int steps1 = get_valence(bb, dir, ok);
    if(steps1 >= OINF) return ok;
    if(setit) setbarrier(current, l1, l2, true);

    if(steps1 % 2 == 0) {
      int s1 = steps1 / 2;
      surround_by(setit, bb, dir, 1, s1, l1, false, ok);
      surround_by(setit, bb, dir, s1+1, steps1-1, l2, true, ok);
      spin_around_by(bb, dir, s1);
      ws = dir > 0 ? NOWALLSEP_WALL_CPOS : NOWALLSEP_WALL_CNEG;
      return ok;
      }

    int s1 = (steps1 - 1) / 2;
    surround_by(setit, bb, dir, 1, s1, l1, false, ok);
    surround_by(setit, bb, dir, s1+2, steps1-1, l2, true, ok);
    spin_around_by(bb, dir, s1);
    bb += dir;
    ws = NOWALLSEP_WALL;
    }
  else if(warped_version(l1, l2)) {
    bb = bb + wstep + (2*dir) + wstep + dir;
    dir = -dir;
    swap(l1, l2);
    }
  else {    
    again:
    cellwalker bb1 = bb;
    int steps = get_valence(bb, dir, ok);
    if(steps >= OINF) return ok;
    int s = 2;
    if(ws == NOWALLSEP_SWAP) s = 5 - s;
    if(dir == -1) s = 5 - s;
    s = (1 + steps - s) / 2;
    surround_by(setit, bb, dir, 1, s, l1, false, ok);
    surround_by(setit, bb, dir, s+2, steps-1, l2, true, ok);
    spin_around_by(bb, dir, s);
    bb += dir;
    if(steps & 1) ws = (ws == NOWALLSEP ? NOWALLSEP_SWAP : NOWALLSEP);
    if(bb.at == bb1.at) goto again;
    }
  return ok;
  }

EX bool general_barrier_check(cellwalker bb, int q, int dir, eLand ws, eLand l1 IS(laNone), eLand l2 IS(laNone)) {

  if(l1 == l2) {
    if(bb.at->mpdist < BARLEV || bb.cpeek()->mpdist < BARLEV || bb.cpeek()->bardir != NODIR || bb.at->bardir != NODIR)
      return false;
    for(int i=0; i<bb.at->type; i++) {
      cell *c1 = bb.at->move(i);
      if(!c1) continue;
      if(c1->bardir != NODIR) return false;
      }
    }
  
  if(l1 != l2 && bb.at->barleft != NOWALLSEP_USED) {
    bb.at->bardir = bb.spin; bb.at->barright = l2; bb.at->barleft = ws; 
    setland(bb.at, l1);
    }
  if(q <= 0) return true;
  
  bool b = general_barrier_advance(bb, dir, l1, l2, ws, l1 != l2);
  if(l1 == l2 && !b) return false;
  return general_barrier_check(bb, q-1, dir, ws, l1, l2);
  }

EX bool general_barrier_check_after(cellwalker bb, int steps, int q, int dir, eLand ws, eLand l1 IS(laNone), eLand l2 IS(laNone)) {
  for(int i=0; i<steps; i++) general_barrier_advance(bb, dir, l1, l2, ws, l1 != l2);
  return general_barrier_check(bb, q, dir, ws, l1, l2);
  }

EX eWall getElementalWall(eLand l) {
  if(l == laEAir) return waChasm;
  if(l == laEEarth) return waStone;
  if(l == laEFire) return waEternalFire;
  if(l == laEWater) return waSea;
  return waNone;
  }

EX void setbarrier(cell *c, eLand l1, eLand l2, bool setbar) {
  if(isSealand(l1) && isSealand(l2)) {
    if(l1 == laKraken || l2 == laKraken)
    if(l1 != laWarpSea && l2 != laWarpSea)
      setbar = !setbar;
    c->wall = setbar ? waBarrier : waSea;
    c->land = laOceanWall;
    }
  else if(isElemental(l1) && isElemental(l2)) {
    c->land = laElementalWall;
    c->wall = getElementalWall(l1);
    }
  else if(l1 == laHaunted || l2 == laHaunted) {
    c->land = laHauntedWall;
    }
  else if(l1 == laMirrored2 || l2 == laMirrored2)
    c->land = laMirrorWall2;
  else if(l1 == laMirrored || l2 == laMirrored)
    c->land = laMirrorWall;
  else if(l1 == laTerracotta && l2 == laTerracotta) {
    c->land = laMercuryRiver;
    c->wall = waMercury;
    }
  else {
    c->wall = waBarrier;
    c->land = laBarrier;
    }
  }

EX void setbarrier(cell *c) {
  setbarrier(c, c->barleft, c->barright, ctof(c));
  }

EX int setland_max = 5;

EX void setland(cell *c, eLand l) {
  if(c->land != l)  {
    c->landparam = 0;  
    }
  if(l == laNone && setland_max > 0) {
    setland_max--;
    printf("error: set land to laNone\n"); // NONEDEBUG
    }
  c->land = l;
  }

EX void extendcheck(cell *c) {
  return;
  if(BITRUNCATED && c->landparam == 0 && c->barleft != NOWALLSEP) {
    raiseBuggyGeneration(c, "extend error");
    }
  }

EX bool mirrorwall(cell *c) {
  return c->barleft == laMirrored || c->barright == laMirrored;
  }
  
EX void extendBarrierFront(cell *c) {
  limitgen("extend front %p\n", hr::voidp(c)); 
  if(buggyGeneration) return;
  int ht = c->landparam;
  extendcheck(c);

  cellwalker bb(c, c->bardir); setbarrier(bb.at);
  bb += wstep;
  
  if(BITRUNCATED) {
    bb.at->barleft = c->barleft;
    bb.at->barright = c->barright;
    setbarrier(bb.at);
    if(!mirrorwall(bb.at)) 
      bb.at->landparam = (ht-4);
  //printf("[A heat %d]\n", ht-4);
  
    setland((bb + 2).cpeek(), c->barleft);
    setland((bb + 4).cpeek(), c->barright);
    
    bb = bb + 3 + wstep; 
    bb.at->barleft = c->barright;
    bb.at->barright = c->barleft;
    setbarrier(bb.at);
    if(!mirrorwall(bb.at)) 
      bb.at->landparam = (ht-4)^2;
  //printf("[B heat %d]\n", (ht-4)^2);
    
    bb = bb + 3 + wstep;
    
    bb.at->barleft = c->barleft;
    bb.at->barright = c->barright;
    if(!mirrorwall(bb.at)) 
      bb.at->landparam = ht ^ 2;
    }

//printf("[C heat %d]\n", (ht)^2);
  bb.at->bardir = bb.spin;
  bb.at->barleft = c->barright;
  bb.at->barright = c->barleft;
  // printf("#1\n");
  extendcheck(bb.at);
  extendBarrier(bb.at);
  
  for(int a=-3; a<=3; a++) if(a) {
    bb.at = c; bb.spin = c->bardir; bb += (PURE?-a:a); bb += wstep; 
    setland(bb.at, a > 0 ? c->barright : c->barleft);
    }
  }

EX void extendBarrierBack(cell *c) {
  limitgen("extend back %p\n", hr::voidp(c)); 
  if(buggyGeneration) return;
  int ht = c->landparam;
  extendcheck(c);

  cellwalker bb(c, c->bardir); setbarrier(bb.at);
  bb = bb + 3 + wstep + (PURE?5:4); 
  setland(bb.at, PURE ? c->barleft : c->barright); 
  bb = bb + wstep + 3;
  bb.at->bardir = bb.spin;
  bb.at->barleft = c->barright;
  bb.at->barright = c->barleft;
  if(!mirrorwall(bb.at)) 
    bb.at->landparam = ht ^ 11;
  extendcheck(bb.at);
//printf("[D heat %d]\n", (ht^11));

  // needed for CR2 to work
  if(BITRUNCATED) { 
    auto bb2 = bb + wstep;
    bb2.at->barleft = c->barright;
    bb2.at->barright = c->barleft;
    if(!mirrorwall(bb2.at)) 
      bb2.at->landparam = (ht^11)-4;
    }
//printf("[E heat %d]\n", (ht^11));

  // printf("#2\n");
  extendBarrier(bb.at);
  }

EX void general_barrier_extend(cell *c) {

  eLand ws = c->barleft;
  cellwalker cw(c, c->bardir);

  c->barleft = NOWALLSEP_USED;
  eLand l1 = c->land;
  eLand l2 = c->barright;

  if(!on_wall(ws)) {
    if(c->bardir == NODIR) {
      println(hlog, "error: NODIR barrier at ", c);
      return;
      }
    setland(cw.cpeek(), l2);
    cw.cpeek()->barleft = NOWALLSEP_USED;
    }

  if(ws == NOWALLSEP_WALL && barrier_cross(l1, l2)) {

    cellwalker p_cw = cw;
    eLand p_l1 = l1, p_l2 = l2;
    eLand p_ws = ws;
    int i = 1;
    general_barrier_advance(p_cw, i, p_l1, p_l2, p_ws, false);

    cellwalker n_cw = cw;
    eLand n_l1 = l1, n_l2 = l2;
    eLand n_ws = ws;
    i = -1;
    general_barrier_advance(n_cw, i, n_l1, n_l2, n_ws, false);

    int dir = 0;

    println(hlog, "left ", n_cw, " = ", n_cw.at->barleft, " right ", p_cw, " = ", p_cw.at->barleft, " USED = ", NOWALLSEP_USED);

    if(n_cw.at->barleft == NOWALLSEP_USED && p_cw.at->barleft != NOWALLSEP_USED) dir = 1;
    if(p_cw.at->barleft == NOWALLSEP_USED && n_cw.at->barleft != NOWALLSEP_USED) dir = -1;

    if(dir) {
      if(!general_barrier_check_after(cw, 2, 10, 1, NOWALLSEP_WALL_EPOS, l1, l1)) {
        println(hlog, "failed to check 1");
        dir = 0;
        }
      if(!general_barrier_check_after(cw+wstep, 2, 10, 1, NOWALLSEP_WALL_EPOS, l1, l1)) {
        println(hlog, "failed to check 2");
        dir = 0;
        }
      }

    if(dir) {
      eLand xl1 = oppositeElement(l1, l2);
      eLand xl2 = oppositeElement(l2, l1);

      if(dir == 1) {
        general_barrier_check_after(cw, 0, 10, 1, NOWALLSEP_WALL_EPOS, l1, xl2);
        general_barrier_check_after(cw+wstep, 0, 10, 1, NOWALLSEP_WALL_EPOS, xl1, l2);
        }
      else {
        general_barrier_check_after(cw, 0, 10, 1, NOWALLSEP_WALL_EPOS, xl2, l1);
        general_barrier_check_after(cw+wstep, 0, 10, 1, NOWALLSEP_WALL_EPOS, l2, xl1);
        }

      general_barrier_check(cw, 10, dir, ws, xl2, xl1);
      return;
      }
    }

  for(int i: {-1, 1}) {  

    if(i == -1 && among(ws, NOWALLSEP_WALL_CPOS, NOWALLSEP_WALL_EPOS)) continue;
    if(i == +1 && among(ws, NOWALLSEP_WALL_CNEG, NOWALLSEP_WALL_ENEG)) continue;

    // general_barrier_check((cw, 10, i, ws, l1, l2);

    cellwalker cw0 = cw;
    eLand xl1 = l1, xl2 = l2;
    eLand ws1 = ws;
    int i1 = i;
    general_barrier_advance(cw0, i1, xl1, xl2, ws1, true);

    if(cw0.at->barleft != NOWALLSEP_USED) {
      setland(cw0.at, xl1);
      cw0.at->barleft = ws1;
      cw0.at->barright = xl2;
      cw0.at->bardir = cw0.spin;
      extendcheck(cw0.at);
      extendBarrier(cw0.at);
      }
    }
  }

bool gotit = false;

EX void extendCR5(cell *c) {
  if(!BITRUNCATED) return;
// if(c->barright == laCrossroads5) extendCR5(c);
  eLand forbidden = c->barleft;
  eLand forbidden2 = laNone;
  cellwalker cw(c, c->bardir);
  for(int u=0; u<2; u++) {
    // if(gotit) break;
    cw = cw + 2 + wstep + 2 + wstep + 5;
    if(cw.at->bardir == NODIR) {
      cw.at->landparam = 40;
      cw.at->bardir = cw.spin;
      cw.at->barright = laCrossroads5;
      eLand nland = forbidden;
      for(int i=0; i<10 && (nland == forbidden || nland == forbidden2); i++)
        nland = getNewLand(laCrossroads5);
      if(ls::single() && specialland == laCrossroads5)
        nland = laCrossroads5;
      cw.at->barleft = forbidden2 = nland;
      landcount[nland]++;
      extendBarrier(cw.at);
      gotit = true;
      }
    else forbidden2 = cw.at->barleft;
    }
  }

EX bool isbar4(cell *c) {
  return
    c->wall == waBarrier || c->land == laElementalWall || 
    c->land == laMirrorWall || c->land == laMirrorWall2 ||
    c->land == laMercuryRiver;
  }  

EX bool barrier_cross(eLand l, eLand r) {
  if(land_structure == lsVineWalls) return false;
  if(l == laCrossroads3 || r == laCrossroads3) return hrand(100) < 66;
  if(land_structure == lsCrossWalls && !among(laCrossroads2, l, r)) return hrand(100) < 90;
  if(isElemental(l) && isElemental(r)) return hrand(100) < 75;
  return false;
  }

EX void extendBarrier(cell *c) {
  limitgen("extend barrier %p\n", hr::voidp(c)); 
  if(buggyGeneration) return;
  
  if(c->barleft == NOWALLSEP_USED) return;

  extendcheck(c);
  
  // printf("build barrier at %p", hr::voidp(c));
  if(c->land == laBarrier || c->land == laElementalWall || c->land == laHauntedWall || c->land == laOceanWall || 
    c->land == laMirrorWall || c->land == laMirrorWall2 || c->land == laMercuryRiver) { 
    // printf("-> ready\n");
    return;
    }
//  if(c->wall == waWaxWall) return;
  if(c->mpdist > BARLEV) {
    // printf("-> too far\n");
    return; // == INFD) return;
    }

  if(c->barleft == NOWALLSEP || c->barleft == NOWALLSEP_SWAP || c->barleft == NOWALLSEP_WALL || on_wall(c->barleft)) {
    #if MAXMDIM >= 4
    if(WDIM == 3) extend3D(c);
    else 
    #endif
    general_barrier_extend(c);
    return;
    }
  
  bool firstmirror = 
    (c->barleft == laMirrored || c->barright == laMirrored) && 
      c->barleft != laMirrored2 && c->barright != laMirrored2;
  
  if(firstmirror && c->barleft == laMirror && hrand(100) < 60) {
    cellwalker cw(c, c->bardir);
    if(BITRUNCATED) cw += wstep;
    if(cw.at->land != laMirrorWall)
      if(buildBarrier6(cw, 1)) return;
    }
    
  if(firstmirror && (PURE?c->barleft == laMirror : c->barright == laMirror) && hrand(100) < 60) {
    cellwalker cw(c, c->bardir);
    if(PURE) {
      cw = cw - 3 + wstep - 3;
      }
    else {
      cw = cw + wstep + 3 + wstep - 1; // check this
      }
    if(buildBarrier6(cw, 2)) return;    
    }
    
  if(land_structure == lsCursedWalls && c->barleft != laMirror && c->barright != laMirror && hrand(100) < 80 && !among(laCrossroads2, c->barleft, c->barright)) {
    cellwalker cw(c, c->bardir);
    cw = cw + wstep + 3 + wstep - 1;
    if(buildBarrier6(cw, c->barright, c->barleft)) return;
    }

  if(barrier_cross(c->barleft, c->barright) || (firstmirror && hrand(100) < 60)) {
    
    cellwalker cw(c, c->bardir);
    if(PURE) {
      cw += wstep;
      if(isbar4(cw.at)) {
        cw = cw + wstep + 3 + wstep - 1 + wstep;
        bool b = buildBarrier4(cw.at, cw.spin, 2, oppositeElement(c->barleft, c->barright), c->barright);
        if(b) return;
        }
      else {
        bool b = buildBarrier4(c, c->bardir, 1, c->barleft, c->barright);
        if(b) return;
        }
      }
    else {
      cw = cw + 3 + wstep;
      cell *cp = (cw + 4 + wstep).at;
      if(!isbar4(cp)) {
        cw = cw + 2 + wstep;
        bool b = buildBarrier4(cw.at, cw.spin, 2, oppositeElement(c->barleft, c->barright), c->barright);
        if(b) return;
        }
      else {
        bool b = buildBarrier4(c, c->bardir, 1, c->barleft, c->barright);
        if(b) return;
        }
      }
    }
  
  extendBarrierFront(c);  
  extendBarrierBack(c);
  
  if(c->barright == laCrossroads5) extendCR5(c);
  }

EX void buildBarrierForce(cell *c, int d, eLand l) {
  c->bardir = d;
  eLand oldland = c->land;
  if(oldland == laNone) {
    raiseBuggyGeneration(c, "oldland is NONE");
    return;
    }
  eLand newland = l ? l : getNewLand(oldland);
  landcount[newland]++;
  if(d == 4 || d == 5 || d == 6) c->barleft = oldland, c->barright = newland;
  else c->barleft = newland, c->barright = oldland;
  if(!mirrorwall(c)) c->landparam = 40;
  extendcheck(c);
  }

EX void buildBarrier(cell *c, int d, eLand l IS(laNone)) {
  
  if(!old_nice_walls()) {
    general_barrier_build(NOWALLSEP_WALL, c, l ? l : getNewLand(c->land), NODIR);
    return;
    }

  d %= 7;
  cellwalker bb(c, d);
  
  if(checkBarriersFront(bb) && checkBarriersBack(bb)) 
    buildBarrierForce(c, d, l);
  }

/** mirror variant of 6-fold walls */
EX bool buildBarrier6(cellwalker cw, int type) {
  limitgen("build6 %p/%d (%d)\n", hr::voidp(cw.at), cw.spin, type); 
  
  cellwalker b[4];
  
  if(buggyGeneration) return true;

  if(BITRUNCATED) {
    b[0] = cw + wstep;
    b[1] = cw + 1 + wstep + 3 + wstep;
    b[2] = cw + 4 + wstep;
    b[3] = cw + 3 + wstep + 3 + wstep;
    }
  else {
    b[0] = cw;
    b[1] = cw + 3 + wstep + 3;
    b[2] = cw - 2 + wstep - 3;
    b[3] = cw - 3 + wstep + 2 + wstep - 3;

    if(type == 1 && b[3].at->land != laMirrorWall) return false;
    if(type == 2 && (b[1] + wstep).at->land != laMirrorWall) return false;
    // if(type == 2 && b[2].at->land != laMirrorWall) return false;
    }
  
  if(false) {
    for(int z=0; z<4; z++) {
      printf("%p/%d\n", hr::voidp(b[z].at), b[z].spin);
      b[z].at->wall = waStrandedBoat; b[z].at->land = laAlchemist;
      b[z].at->mondir = b[z].spin;
      b[z].at->mpdist = 7;
      b[z].at->item = eItem(1+z);
      buggyGeneration = true;
      }
    return true;
    }
   
  if(type == 1) {  
    if(!(PURE?checkBarriersFront:checkBarriersBack)(b[1], 6, true)) return false;
    if(!(PURE?checkBarriersFront:checkBarriersBack)(b[2], 6, true)) return false;
    }
  else {
    if(!(PURE?checkBarriersFront:checkBarriersBack)(b[0], 6, true)) return false;
    if(!(PURE?checkBarriersFront:checkBarriersBack)(b[3], 6, true)) return false;
    }

  eLand m0 = laMirror;
  eLand m1 = laMirrored;
  eLand m2 = laMirrored2;
  eLand mw1 = laMirrorWall;
  eLand mw2 = laMirrorWall2;
  eWall w = waMirrorWall;

  for(int d=0; d<4; d++) {
    b[d].at->bardir = b[d].spin;
    
    if(PURE) {
      b[0].at->barleft = m1, b[0].at->barright = m2;
      b[1].at->barleft = m0, b[1].at->barright = m1;
      b[2].at->barleft = m2, b[2].at->barright = m1;
      b[3].at->barleft = m1, b[3].at->barright = m0;
      }
    else {    
      b[0].at->barleft = m0, b[0].at->barright = m1;
      b[1].at->barleft = m1, b[1].at->barright = m0;
      b[2].at->barleft = m1, b[2].at->barright = m2;
      b[3].at->barleft = m2, b[3].at->barright = m1;
      }
  
    (PURE?extendBarrierFront:extendBarrierBack)(b[d].at);
    }  

  if(PURE && false) {
    for(int z=0; z<4; z++)
      b[z].at->item = eItem(1+z+4*type);
    for(int a=0; a<4; a++) 
      extendBarrierBack((b[a]+wstep).at);
    }

  if(BITRUNCATED) {
    setland((cw+1).cpeek(), mw1);
    setland((cw+2).cpeek(), m1);
    setland((cw+3).cpeek(), mw2);
    setland((cw+4).cpeek(), mw2);
    setland((cw+5).cpeek(), m1);
    setland((cw+0).cpeek(), mw1);
    setland((b[0]+2).cpeek(), m1);
    setland((b[3]+6).cpeek(), m2);
    setland((b[3]+5).cpeek(), m2);
    setland((b[1]-1).cpeek(), m1);
    setland((b[2]-2).cpeek(), m1);
    setland((b[1]-2).cpeek(), m1);
    setland((b[0]-2).cpeek(), m0);
    cw.at->land = mw1;
    cw.at->wall = w;
    cw.at->landparam = 1;
    }
  else {
    setland(cw.at, mw2);
    setland((cw+0).cpeek(), mw2);
    setland((cw+1).cpeek(), m1);
    setland((cw+2).cpeek(), m1);
    setland((cw+3).cpeek(), mw1);
    setland((cw+4).cpeek(), m1);
    setland((cw+5).cpeek(), mw2);
    setland((cw+6).cpeek(), m2);

    setland((b[1]).cpeek(), mw1);
    setland((b[1]+1).cpeek(), m0);
    setland((b[1]+2).cpeek(), mw1);
    setland((b[1]+6).cpeek(), m1);

    setland((b[0] + wstep - 2).cpeek(), m1);
    setland((b[3] + wstep - 2).cpeek(), m1);
    }

  return true;
  }

EX int curse_percentage = 10;

/** cursed variant of 6-fold walls */
EX bool buildBarrier6(cellwalker cw, eLand m0, eLand m1) {
  cellwalker b[6];
  if(buggyGeneration) return true;

  for(int i=0; i<6; i+=2)
    b[i] = cw + i + wstep;
  for(int i=1; i<6; i+=2)
    b[i] = cw + i + wstep + 3 + wstep;

  for(int i=0; i<6; i++) if(i != 1) {
    if(!(PURE?checkBarriersFront:checkBarriersBack)(b[i], 6, true)) return false;
    }

  for(int d=0; d<6; d++) {
    b[d].at->bardir = b[d].spin;
    b[d].at->barleft = (d&1) ? m1 : m0;
    b[d].at->barright = (d&1) ? m0 : m1;
    (PURE?extendBarrierFront:extendBarrierBack)(b[d].at);
    }

  cw.at->land = laBarrier;
  cw.at->wall = waBarrier;
  forCellCM(c, cw.at) { c->land = laBarrier; c->wall = waBarrier; }
  for(int d=0; d<6; d+=2) {
    setland((b[d]-2).cpeek(), m0);
    setland((b[d]+2).cpeek(), m1);
    setland((b[d+1]-2).cpeek(), m1);
    setland((b[d+1]+2).cpeek(), m0);
    }
  if(hrand(100) < curse_percentage) {
    setland(cw.at, laCursed);
    cw.at->wall = waRubble;
    cw.at->monst = moHexer;
    cw.at->item = random_curse();
    }

  return true;
  }

EX bool buildBarrier4(cell *c, int d, int mode, eLand ll, eLand lr) {
  limitgen("build4 %p\n", hr::voidp(c)); 
  if(buggyGeneration) return true;
  d %= 7;
  
  cellwalker cd(c, d);
  
  cellwalker b1 = cd;
  cellwalker b2 = PURE ? cd + wstep : cd + wstep + 3 + wstep + 3 + wstep;  
  cellwalker b3 = PURE ? cd - 1 + wstep + 3 : cd + wstep + 4 + wstep + 4;
  cellwalker b4 = PURE ? cd + 1 + wstep - 3 : cd + wstep - 4 + wstep - 4;
  
  if(mode == 0) {
    if(!((checkBarriersBack(b1) && checkBarriersBack(b2)))) return false;
    if(!((checkBarriersFront(b3) && checkBarriersFront(b4)))) return false;
    }

  if(mode == 1) {
    if(!(checkBarriersFront(b3, 5, true) && checkBarriersFront(b4, 5, true))) 
      return false;
    }
    
  if(mode == 2) {
    if(!((checkBarriersBack(b1, 5, true) && checkBarriersBack(b2, 5, true)))) 
      return false;
    }
  
  eLand xl = oppositeElement(ll, lr);
  eLand xr = oppositeElement(lr, ll);

  c->bardir = d, c->barleft = ll, c->barright = lr; extendBarrierBack(c);
  
  c= b2.at; d=b2.spin;
  c->bardir = d, c->barleft = xl, c->barright = xr; extendBarrierBack(c);

  c= b3.at; d=b3.spin;
  c->bardir = d, c->barleft = xl, c->barright = lr; extendBarrierFront(c);

  c= b4.at; d=b4.spin;
  c->bardir = d, c->barleft = ll, c->barright = xr; extendBarrierFront(c);
  
  if(BITRUNCATED) for(int a=-3; a<=3; a++) if(a) {
    setland((b1+a).cpeek(), a > 0 ? lr : ll);
    setland((b2+a).cpeek(), a > 0 ? xr : xl);
    setland((b3+a).cpeek(), a > 0 ? lr : xl);
    setland((b4+a).cpeek(), a > 0 ? xr : ll);
    }
  
  if(PURE) setbarrier(b1.at), setbarrier(b2.at), setbarrier(b3.at), setbarrier(b4.at);

  if(BITRUNCATED) {
    cell *cp;
    cp = (b1+wstep).at;
    cp->barleft = ll; cp->barright = lr; setbarrier(cp);
    cp = (b2+wstep).at;
    cp->barleft = xl; cp->barright = xr; setbarrier(cp);
    }
  
  return true;
  }    

EX void buildBarrierStrong(cell *c, int d, bool oldleft, eLand newland) {
  d %= 7;
  cellwalker bb(c, d);

  c->bardir = d;
  eLand oldland = c->land;
  landcount[newland]++;

  if(oldleft) c->barleft = oldland, c->barright = newland;
  else c->barleft = newland, c->barright = oldland;
  extendcheck(bb.at);
  }

EX void buildBarrierStrong(cell *c, int d, bool oldleft) {
  buildBarrierStrong(c, d, oldleft, getNewLand(c->land));
  }

EX void buildCrossroads2(cell *c) {

  if(buggyGeneration) return;

  if(!c) return;
  if(ls::hv_structure()) return;
  
  for(int i=0; i<c->type; i++)
    if(c->move(i) && !c->move(i)->landparam && c->move(i)->mpdist < c->mpdist)
      buildCrossroads2(c->move(i));
  
  if(hasbardir(c))
    extendBarrier(c);

  if(c->land != laCrossroads2) return;

  if(!c->landparam) {
    for(int i=0; i<c->type; i++) { 
      cell *c2 = createMov(c, i);
      if(c2 && c2->landparam && (c2->land == laCrossroads2 || c2->land == laBarrier)) {
        for(int j=0; j<c2->type; j++) {
          createMov(c2, j);
          cell *c3 = c2->move(j);
          if(c3 && c3->landparam && (c3->land == laCrossroads2 || c3->land == laBarrier)) {
            int h2 = c2->landparam;
            int h3 = c3->landparam;
            
            if(h2 > 100) { raiseBuggyGeneration(c2, "bad c2 landparam"); return; }
            if(h3 > 100) { raiseBuggyGeneration(c3, "bad c3 landparam"); return; }
            
            // ambiguous
            if(h2/4 == 1 && h3/4 == 3) continue;
            if(h2/4 == 3 && h3/4 == 1) continue;
            
            for(int d=0; d<c2->type; d++)
              if(emeraldtable[h2][d] == h3) {
                int nh = emeraldtable[h2][(42+d + c->c.spin(i) - j) % c2->type];
                if(c->landparam>0 && c->landparam != nh) { 
                  printf("CONFLICT\n"); 
                  raiseBuggyGeneration(c, "CONFLICT"); 
                  }
                c->landparam = nh;
                }
    
            if(c->landparam == 0) 
              printf("H2 = %d H3=%d\n", c2->landparam, c3->landparam);
            }
          }
        if(c->landparam == 0) {
          printf("H2 = %d\n", c2->landparam);
//        halted = true;
//        c->heat = -1;
          raiseBuggyGeneration(c, "buildCrossroads2x");
          return;
          }
        }
      }
    
    if(c->landparam) {
//    for(int i=0; i<c->type; i++) { 
//      cell *c2 = c->move(i);
//      buildCrossroads2(c2);
//      }
      }
    else {
      raiseBuggyGeneration(c, "buildCrossroads2");
      return;
      }
    }

  int h = c->landparam;
  
  if(h/4 >= 8 && h/4 <= 11) {
    for(int i=0; i<c->type; i++) if(c->land != laBarrier) { 
      cell *c2 = createMov(c, i);
      if(c2->land == laBarrier) continue;
      c2->land = laCrossroads2;
      if(!c2->landparam) buildCrossroads2(c2);
      }
    if(h/4 == 8 || h/4 == 10)
    for(int i=0; i<c->type; i++) { 
      if(c->move(i) && c->move(i)->landparam == h-4) {
        bool oldleft = true;
        for(int j=1; j<=3; j++) 
          if(c->modmove(i+j) && c->modmove(i+j)->mpdist < c->mpdist)
            oldleft = false;
            
        c->landparam = h;
        if(ls::single())
          buildBarrierStrong(c, i, oldleft, specialland);
        else
          buildBarrierStrong(c, i, oldleft);
        c->landparam = h;
        extendBarrier(c);
        }
      }
    }
  }

#if MAXMDIM >= 4
EX bool bufferzone() { return PURE && S7 == 6; }
EX int basic_tests() { return 50; }

EX bool valid_dir(const vector<char>& ad, int j, cell *c) {
  bool bch = variation == eVariation::bch;
  if(!bch) return ad[j] == 1;

  if(ad[j] != 2) return false;
  auto ad1 = currentmap->get_cellshape(c).dirdist[j];
  int a = 0;
  for(auto& dd: ad1) if(dd == 1) a++;

  return a < 6;
  }

EX void extend3D(cell *c) {
  eLand l1 = c->land;
  c->barleft = NOWALLSEP_USED;
  
  cellwalker cw(c, c->bardir);
  
  if(bufferzone()) {
    cw += wstep; cw += rev;
    cw.at->bardir = NOBARRIERS;
    setland(cw.at, laBarrier);
    }
  
  auto cw1 = cw + wstep;
  setland(cw1.at, c->barright);
  if(cw1.at->bardir == NODIR) {
    cw1.at->barleft = NOWALLSEP_USED;
    cw1.at->barright = l1;
    cw1.at->bardir = cw1.spin;
    }

  bool bch = variation == eVariation::bch;

  auto& ad = currentmap->dirdist(cw);
  for(int j=0; j<cw.at->type; j++) {
    
    if(!valid_dir(ad, j, cw.at)) {
      if(bch && ad[j] == 1) {
        cell *c1 = cw.at->cmove(j);
        c1->bardir = NOBARRIERS;
        setland(c1, c->barright);
        }
  
      if(bch && ad[j] == 2) {
        cell *c1 = cw.at->cmove(j);
        c1->bardir = NOBARRIERS;
        setland(c1, l1);
        }
  
      continue;
      }
    
    cellwalker bb2 = currentmap->strafe(cw, j);
    if(bufferzone()) { bb2 += rev; bb2 += wstep; }

    if(bb2.at->bardir == NODIR) {
      bb2.at->bardir = bb2.spin;
      bb2.at->barleft = NOWALLSEP;
      bb2.at->barright = c->barright;
      bb2.at->land = l1;
      // bb2.at->item = itGold;
      extendBarrier(bb2.at);
      }
    }
  }

bool built = false;

EX bool buildBarrier3D(cell *c, eLand l2, int forced_dir) {
  if(forced_dir == NODIR) {
    for(int t=0; t<c->type; t++) if((!c->move(t) || c->move(t)->mpdist > c->mpdist) && buildBarrier3D(c, l2, t)) return true;
    return false;
    }
  bool bch = variation == eVariation::bch;
  
  cellwalker cw(c, forced_dir);

  if(bch) {
    auto& ad = currentmap->dirdist(cw);
    int a = 0;
    for(auto& dd: ad) if(dd == 1) a++;
    if(a == 6) return false;
    }
  
  if(bufferzone()) { cw += wstep; cw += rev; }
  set<cell*> listed_cells = { cw.at };
  vector<cellwalker> to_test { cw };
  int tc = basic_tests();
  for(int i=0; i<isize(to_test); i++) {
    auto bb = to_test[i];
    if(bb.at->mpdist < BARLEV) return false;
    if(bb.cpeek()->mpdist < BARLEV) return false;
    if(bb.cpeek()->bardir != NODIR) return false;
    if(bufferzone() && (bb+rev).cpeek()->mpdist < BARLEV) return false;
    if(bufferzone() && (bb+rev).cpeek()->bardir != NODIR) return false;
    if(bb.at->bardir != NODIR) return false;
    auto& ad = currentmap->dirdist(bb);
    for(int j=0; j<bb.at->type; j++) {
      if(i < tc) bb.at->cmove(j);
      if(!bb.at->move(j)) continue;
      if(!valid_dir(ad, j, bb.at)) continue;
      cellwalker bb2 = currentmap->strafe(bb, j);
      if(listed_cells.count(bb2.at)) continue;
      listed_cells.insert(bb2.at);
      to_test.push_back(bb2);
      }
    }

  for(int i=0; i<isize(to_test); i++) {
    auto bb = to_test[i];
    if(bufferzone()) { bb.at->bardir = NOBARRIERS; setland(bb.at, laBarrier); bb += rev; bb += wstep; }
    bb.at->land = c->land;
    bb.at->bardir = bb.spin;
    bb.at->barleft = NOWALLSEP;
    bb.at->barright = l2;
    extendBarrier(bb.at);
    }

  built = true;
  return true;
  }
#endif

EX bool buildBarrierNowall(cell *c, eLand l2, int forced_dir IS(NODIR)) {
  if(among(l2, laCrossroads2, laCrossroads5)) return false;
  return general_barrier_build(NOWALLSEP, c, l2, forced_dir);
  }

EX bool general_barrier_build(eLand ws, cell *c, eLand l2, int forced_dir IS(NODIR)) {

  if(S3 >= OINF) { c->land = l2; return true; }

  #if MAXMDIM >= 4
  // 3D binary tilings create walls using their own methods
  if(WDIM == 3 && bt::in()) return false;

  if(WDIM == 3 && hyperbolic) return buildBarrier3D(c, l2, forced_dir);
  #endif

  if(c->land == laNone) {
    printf("barrier nowall! [%p]\n", hr::voidp(c));
    raiseBuggyGeneration(c, "barrier nowall!");
    return false;
    }
  
  bool warpv = warped_version(c->land, l2);
  if(warpv && !arcm::in() && !pseudohept(c)) return false;
  
  vector<int> ds = hrandom_permutation(c->type);
  
  eLand wsx = warpv ? laWarpCoast : laNone;
  eLand l1 = c->land;
  
  if(forced_dir != NODIR) {
    cellwalker cw(c, forced_dir);
    general_barrier_check(cw, 20, -1, ws, l1, l2);
    general_barrier_check(cw, 20, 1, ws, l1, l2);
    extendBarrier(c);
    return true;
    }

  for(int d: ds) {
    if(c->move(d) && c->move(d)->mpdist <= c->mpdist) continue;    
    cellwalker cw(c, d);    
    if(general_barrier_check(cw, 20, -1, ws, wsx, wsx) && general_barrier_check(cw, 20, 1, ws, wsx, wsx)) {
      general_barrier_check(cw, 20, -1, ws, l1, l2);
      general_barrier_check(cw, 20, 1, ws, l1, l2);
      extendBarrier(c);
      return true;
      }
    }
  
  return false;
  }

}
