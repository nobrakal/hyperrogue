// Hyperbolic Rogue - checkmate rule
// Copyright (C) 2011-2019 Zeno Rogue, see 'hyper.cpp' for details

/** \file checkmove.cpp
 *  \brief Check the validity of move (checkmate rule)
 */

#include "hyper.h"

namespace hr {

#if HDR
#define PUREHARDCORE_LEVEL 10
#endif

/** are we in the hardcore mode */
EX bool hardcore = false;
/** when did we switch to the hardcore mode */
EX int hardcoreAt;
/** are we in the casual mode */
EX bool casual = false;

EX bool pureHardcore() { return hardcore && hardcoreAt < PUREHARDCORE_LEVEL; }

/** can we still move? */
EX bool canmove = true;

// how many monsters are near
EX eMonster who_kills_me;
EX cell *who_kills_me_cell;

EX int lastkills;

EX vector<bool> legalmoves;

#if HDR
struct moveissue {
  int type;
  int subtype;
  eMonster monster;
  cell *where;
  };

static constexpr int miVALID = 10000;
static constexpr int miENTITY = 11000;
static constexpr int miRESTRICTED = 10100;
static constexpr int miTHREAT = 10010;
static constexpr int miWALL = 10001;

static constexpr int siWALL = 1;
static constexpr int siMONSTER = 2;
static constexpr int siGRAVITY = 3;
static constexpr int siROSE = 4;
static constexpr int siITEM = 5;
static constexpr int siWIND = 6;
static constexpr int siCURRENT = 7;
static constexpr int siFATIGUE = 8;
static constexpr int siWARP = 9;
static constexpr int siUNKNOWN = 10;
#endif

/* why is a move illegal */
EX vector<moveissue> move_issues;

EX moveissue checked_move_issue;
EX moveissue stay_issue;
EX int yasc_code;

EX void check_if_monster() {
  eMonster m = cwt.peek()->monst;
  if(m && m != passive_switch && !isFriendly(m))
    checked_move_issue = moveissue { miENTITY, siMONSTER, m, cwt.peek() };
  }
  
EX bool hasSafeOrb(cell *c) {
  return 
    c->item == itOrbSafety ||
    c->item == itOrbShield ||
    c->item == itOrbShell  ||
    (c->item == itOrbYendor && yendor::state(c) == yendor::ysUnlocked);
  }

#if HDR
struct player_move_info {
  movei mi;
  cell *swordlast[2], *swordtransit[2], *swordnext[2];
  player_move_info(movei mi);
  };
#endif

EX vector<player_move_info> pmi;
EX vector<cell*> pushes;

player_move_info::player_move_info(movei _mi) : mi(_mi) {
  for(int b=0; b<2; b++) swordlast[b] = sword::pos(multi::cpid, b);

  dynamicval<sword::sworddir> x7(sword::dir[multi::cpid], sword::shift(mi, sword::dir[multi::cpid]));
  
  for(int b=0; b<2; b++) {
    swordnext[b] = sword::pos(multi::cpid, b);
    swordtransit[b] = NULL;
    if(swordnext[b] && swordnext[b] != swordlast[b] && !isNeighbor(swordlast[b], swordnext[b])) {
      forCellEx(c2, swordnext[b])
        if(c2 != mi.t && c2 != mi.s && isNeighbor(c2, S3==3 ? swordlast[b] : mi.t))
          swordtransit[b] = c2;
      if(S3 == 4)
        forCellEx(c2, mi.s)
          if(c2 != mi.s && isNeighbor(c2, swordlast[b]))
            swordtransit[b] = c2;
      }
    }
  }

EX bool krakensafe(cell *c) {
  return items[itOrbFish] || items[itOrbAether] || 
    (c->item == itOrbFish && c->wall == waBoat) || 
    (c->item == itOrbAether && c->wall == waBoat);
  }

EX bool monstersnear(cell *c, eMonster who) {

  bool eaten = false;

  if(hardcore && who == moPlayer) return false;

  int res = 0;
  bool fast = false;
  bool kraken_will_destroy_boat = false;

  elec::builder b;
  if(elec::affected(c)) { who_kills_me = moLightningBolt; who_kills_me_cell = nullptr; res++; }
  
  if(c->wall == waArrowTrap && c->wparam == 2) {
    who_kills_me = moArrowTrap; who_kills_me_cell = nullptr; res++;
    }
  
  for(auto c1: crush_now) if(c == c1) {
    who_kills_me = moCrusher; who_kills_me_cell = nullptr; res++;
    }

  if(who == moPlayer || items[itOrbEmpathy]) {
    fast = (items[itOrbSpeed] && (items[itOrbSpeed] & 1));
    if(who == moPlayer && !isPlayerOn(c) && c->item == itOrbSpeed && !items[itOrbSpeed]) fast = true;
    }
  
  if(havewhat&HF_OUTLAW) {
    for(cell *c1: gun_targets(c)) 
      if(c1->monst == moOutlaw && !c1->stuntime) {
        res++; who_kills_me = moOutlaw; who_kills_me_cell = c1;
        }
    }

  for(int t=0; t<c->type; t++) {
    cell *c2 = c->move(t);

    // consider monsters who attack from distance 2
    if(c2) forCellEx(c3, c2) if(c3 != c) {
      // only these monsters can attack from two spots...
      if(!among(c3->monst, moLancer, moWitchSpeed, moWitchFlash)) 
        continue;
      if(c3->monst == moWitchSpeed && cwt.at->land != laPower)
        continue;
      // take logical_adjacent into account
      if(c3->monst != moWitchFlash)
        if(!logical_adjacent(c3, c3->monst, c2) || !logical_adjacent(c2, c3->monst, c) || (c3->monst == moWitchSpeed && c2->land != laPower))
          continue;
      if(elec::affected(c3)) continue;
      if(c3->stuntime > (who == moPlayer ? 0 : 1)) continue;
      // speedwitches can only attack not-fastened monsters,
      // others can only attack if the move is not fastened
      if(c3->monst == moWitchSpeed && items[itOrbSpeed]) continue;
      if(c3->monst != moWitchSpeed && fast) continue;
      // cannot attack if the immediate cell is impassable (except flashwitches)
      if(c3->monst != moWitchFlash) {
        if(!passable(c2, c3, 0)) continue;
        if(isPlayerOn(c2) && items[itOrbFire]) continue;
        }
      // flashwitches cannot attack if it would kill another enemy
      if(c3->monst == moWitchFlash && flashWouldKill(c3, 0)) continue;
      res++, who_kills_me = c3->monst; who_kills_me_cell = c3;
      } 

    // consider normal monsters
    if(c2 && isArmedEnemy(c2, who)) {
      eMonster m = c2->monst;
      if(elec::affected(c2)) continue;
      if(fast && c2->monst != moWitchSpeed) continue;
      // Krakens just destroy boats
      if(who == moPlayer && c2->monst == moKrakenT && c->wall == waBoat && !peace::on) {
        kraken_will_destroy_boat = true;
        continue;
        }
      // they cannot attack through vines
      if(!canAttack(c2, c2->monst, c, who, AF_NEXTTURN)) continue;
      if(c2->monst == moWorm || c2->monst == moTentacle || c2->monst == moHexSnake) {
        if(passable_for(c2->monst, c, c2, 0))
          eaten = true;
        else if(c2->monst != moHexSnake) continue;
        }
      res++, who_kills_me = m; who_kills_me_cell = c2;
      }
    }

  if(kraken_will_destroy_boat && !krakensafe(c) && warningprotection(XLAT("This move appears dangerous -- are you sure?"))) {
    if (res == 0) who_kills_me = moWarning; who_kills_me_cell = nullptr;
    res++;
  } else {
    if(who == moPlayer && res && (markOrb2(itOrbShield) || markOrb2(itOrbShell)) && !eaten)
      res = 0;
    if(who == moPlayer && res && markOrb2(itOrbDomination) && c->monst)
      res = 0;
  }

  return !!res;
  }

EX bool monstersnear_aux() {
  changes.value_set(passive_switch, (gold() & 1) ? moSwitch1 : moSwitch2);
  multi::cpid++;
  bool b = false;
  if(multi::cpid == multi::players || multi::players == 1 || multi::checkonly) {

    if(shmup::delayed_safety) return false;
    
    for(int i=0; i<isize(pmi); i++)
    for(int j=0; j<isize(pmi); j++) if(i != j) {
      if(swordConflict(pmi[i], pmi[j])) {
          b = true;
          who_kills_me = moEnergySword;
          }
      if(pmi[i].mi.t == pmi[j].mi.t) 
        { b = true; who_kills_me = moFireball; }
      if(celldistance(pmi[i].mi.t, pmi[j].mi.t) > 8) 
        { b = true; who_kills_me = moAirball; }
      }

    for(auto& pushto: pushes)
    for(auto& mi: pmi)
      if(pushto == mi.mi.t) {
        b = true; who_kills_me = moTongue;
        }
        
    for(int i=0; i<isize(pushes); i++)
      for(int j=0; j<i; j++)
        if(pushes[i] == pushes[j]) {
          b = true; who_kills_me = moCrushball;
          }
          
    for(int i=0; !b && i<isize(pmi); i++)
      b = monstersnear(pmi[i].mi.t, moPlayer);
    }
  else b = !multimove();
  multi::cpid--;
  return b;
  }

/** like monstersnear but add the potential moves of other players into account */
EX bool monstersnear_add_pmi(player_move_info pmi0) {
  if(suicidal) {
    who_kills_me = moPlayer; who_kills_me_cell = nullptr;
    return true;
    }
  pmi.push_back(pmi0);
  bool b = monstersnear_aux();
  pmi.pop_back();
  return b;
  }

EX bool multimove() {
  if(multi::cpid == 0) lastkills = tkills();
  if(!multi::playerActive(multi::cpid)) return !monstersnear_aux();
  cellwalker bcwt = cwt;
  cwt = multi::player[multi::cpid];
  bool b = movepcto(multi::whereto[multi::cpid]);
  if(b) {
    multi::aftermove = true;
    multi::player[multi::cpid] = cwt;
    multi::whereto[multi::cpid].d = MD_UNDECIDED;
    int curkills = tkills();
    multi::kills[multi::cpid] += (curkills - lastkills);
    lastkills = curkills;
    }
  cwt = bcwt;
  return b;
  }

EX namespace multi { 
  EX bool checkonly = false;
  EX bool aftermove;
  EX }

EX bool swordConflict(const player_move_info& sm1, const player_move_info& sm2) {
  if(items[itOrbSword] || items[itOrbSword2])
  for(int b=0; b<2; b++)
    if(sm1.mi.s == sm2.swordlast[b] || sm1.mi.s == sm2.swordtransit[b] || sm1.mi.s == sm2.swordnext[b])
    if(sm1.mi.t == sm2.swordlast[b] || sm1.mi.t == sm2.swordtransit[b] || sm1.mi.t == sm2.swordnext[b])
      return true;
  return false;
  }

EX string yasc_message;

EX string blocking_monster_name(const moveissue& mi) {
  if(mi.monster == moKnight && mi.where)
    return XLAT("%1 the Knight", camelot::knight_name(mi.where));
  else
    return dnameof(mi.monster);
  }

EX void create_yasc_message() {
  set<pair<cell*, string>> captures;
  auto all = move_issues;
  all.push_back(stay_issue);
  for(auto c: all) if(c.type == miTHREAT) captures.emplace(c.where, blocking_monster_name(c));

  vector<string> context;
  if(!captures.empty()) {
    string msg = "captured by ";
    map<string, int> qties;
    for(auto ca: captures) qties[ca.second]++;
    int iqties = 0;
    for(auto q: qties) {
      if(iqties && iqties == isize(qties) - 1) msg += " and ";
      else if(iqties) msg += ", ";
      msg += q.first;
      if(q.second > 1) msg += " (x" + its(q.second) + ")";
      iqties++;
      }
    context.push_back(msg);
    }

  int idx = 0;
  for(auto c: all) if(idx == 0 && c.subtype == siROSE) context.push_back("rosed"), idx = 1;
  for(auto c: all) if(idx < 2 && c.subtype == siWIND) context.push_back("blown away"), idx = 2;
  for(auto c: all) if(idx < 3 && c.subtype == siGRAVITY) context.push_back("falling"), idx = 3;
  for(auto c: all) if(idx < 4 && c.subtype == siFATIGUE) context.push_back("fatigued"), idx = 4;
  for(auto c: all) if(idx < 5 && c.subtype == siCURRENT) context.push_back("whirled"), idx = 5;

  bool in_ctx = true;

  set<string> blocks;
  for(auto c: all) {
    if(c.type == miENTITY && !captures.count({c.where, blocking_monster_name(c)})) blocks.insert(blocking_monster_name(c));
    else if(c.type == miWALL && c.subtype == siMONSTER && !captures.count({c.where, blocking_monster_name(c)})) blocks.insert(blocking_monster_name(c));
    else if(c.subtype == siITEM) blocks.insert("item");
    else if(c.subtype == siWALL) {
      if(c.where == cwt.at) {
        if(in_ctx) {
          if(c.where->wall == waNone && c.where->land == laBrownian)
            context.push_back("on level 3");
          else if(c.where->wall == waRoundTable)
            context.push_back("being polite");
          else
            context.push_back(winf[c.where->wall].flags & WF_ON ? XLAT("on %the1", c.where->wall) : XLAT("in %the1", c.where->wall));
          }
        in_ctx = false;
        }
      else if(c.where && c.where->wall != cwt.at->wall) blocks.insert(dnameof(c.where->wall));
      }
    else if(c.type == siWARP) blocks.insert("warp");
    }

  if(!blocks.empty()) {
    string block = "blocked by ";
    int iqties = 0;
    for(auto& q: blocks) {
      if(iqties && iqties == isize(blocks) - 1) block += " and ";
      else if(iqties) block += ", ";
      block += q;
      iqties++;
      }
    context.push_back(block);
    }

  yasc_message = "";
  int iqties = 0;
  for(auto& ctx: context) {
    if(iqties == 0) ;
    else if(iqties == 1) yasc_message += " while ";
    else if(iqties == isize(context) - 1) yasc_message += " and ";
    else yasc_message += ", ";
    yasc_message += ctx;
    iqties++;
    }

  if(captures.size() == 2 && context.size() == 1 && cwt.at->type == 6) {
    vector<int> dirs;
    forCellIdEx(c1, i, cwt.at) for(auto cap: captures) if(cap.first == c1) dirs.push_back(i);
    if(isize(dirs) == 2 && abs(dirs[0]-dirs[1]) == 3) {
      auto c1 = captures.begin(); c1++;
      yasc_message = XLAT("pinched by %the1 and %the2", captures.begin()->second, c1->second);
      }
    }

  println(hlog, "YASC_MESSAGE: ", yasc_message);
  }

int yasc_recode(int x) {
  if(cwt.at->type < 10 || x == 0) return x;
  return yasc_recode(x / 10) * 100 + (x % 10);
  };

EX void checkmove() {

  if(dual::state == 2) return;
  if(shmup::on) return;

  dynamicval<eGravity> gs(gravity_state, gravity_state);

#if CAP_INV
  if(inv::on) inv::compute();
#endif

  if(multi::players > 1 && !multi::checkonly) return;
  if(hardcore) return;
  
  legalmoves.clear(); legalmoves.resize(cwt.at->type+1, false);
  move_issues.clear(); move_issues.resize(cwt.at->type);

  canmove = haveRangedTarget();
  items[itWarning]+=2;
  if(movepcto(-1, 0, true))
    canmove = legalmoves[cwt.at->type] = true;
  stay_issue = checked_move_issue;
  
  if(true) {
    for(int i=0; i<cwt.at->type; i++) {
      dynamicval<bool> fm(bow::fire_mode, false);
      if(movepcto(1, -1, true)) {
        canmove = legalmoves[cwt.spin] = true;
        }
      check_if_monster();
      move_issues[cwt.spin] = checked_move_issue;
      if(!legalmoves[cwt.spin]) {
        if(movepcto(0, 1, true)) {
          canmove = legalmoves[cwt.spin] = true;
          }
        check_if_monster();
        move_issues[cwt.spin] = checked_move_issue;
        }
      }
    }
  if(kills[moPlayer]) canmove = false;
  
  yasc_code = 0;
  for(int i=0; i<cwt.at->type; i++)
    yasc_code += yasc_recode(move_issues[i].type);

  if(!canmove && bow::crossbow_mode() && !items[itCrossbow]) canmove = bow::have_bow_target();

#if CAP_INV  
  if(inv::on && !canmove && !inv::incheck) {
    if(inv::remaining[itOrbSafety] || inv::remaining[itOrbFreedom])
      canmove = true;
    else {
      inv::check(1);
      checkmove();
      inv::check(-1);
      }
    if(canmove)
      pushScreen(inv::show);
    }
#endif

  if(!canmove) {
    create_yasc_message();
    achievement_final(true);
    if(cmode & sm::NORMAL) showMissionScreen();
    }
  else yasc_message = "";

  if(canmove && timerstopped) {
    timerstart = time(NULL);
    timerstopped = false;
    }
  items[itWarning]-=2;

  if(recallCell.at && !markOrb(itOrbRecall)) activateRecall();  
  }


}
