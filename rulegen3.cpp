// Hyperbolic Rogue -- rule generator
// Copyright (C) 2011-2021 Zeno Rogue, see 'hyper.cpp' for details

/** \file rulegen3.cpp 
 *  \brief An algorithm to create strict tree rules for arb tessellations -- 3D parts
 */

#include "hyper.h"

namespace hr {

#if MAXMDIM >= 4
EX namespace rulegen {

struct road_shortcut_trie_vertex {
  set<vector<int>> backpaths;
  map<int, shared_ptr<struct road_shortcut_trie_vertex>> children;
  };

EX map<int, shared_ptr<struct road_shortcut_trie_vertex>> road_shortcuts;

int qroad;

map<int, int> qroad_for;
map<tcell*, int> qroad_memo;

EX void add_road_shortcut(tcell *s, tcell *t) {
  if(flags & w_r3_no_road_shortcuts) return;
  shared_ptr<road_shortcut_trie_vertex> u;
  vector<int> tpath;
  if(!road_shortcuts.count(s->id)) road_shortcuts[s->id] = make_shared<road_shortcut_trie_vertex>();
  u = road_shortcuts[s->id];
  while(true) {
    // println(hlog, s, " dist=", s->dist, " parent = ", s->parent_dir, " vs ", t, " dist=", t->dist, " parent = ", t->parent_dir);
    if(s == t) {
      reverse(tpath.begin(), tpath.end());
      auto& ba = u->backpaths;
      if(!ba.count(tpath)) qroad++, qroad_for[s->id]++;
      ba.insert(tpath);
      return;
      }
    if(s->dist >= t->dist) {
      twalker sw = s;
      get_parent_dir(sw);
      if(s->parent_dir == MYSTERY) throw hr_exception("unknown parent_dir (s) in add_road_shortcut");
      if(!u->children.count(s->parent_dir)) u->children[s->parent_dir] = make_shared<road_shortcut_trie_vertex>();
      u = u->children[s->parent_dir];
      s = s->move(s->parent_dir);
      }
    if(t->dist > s->dist) {
      twalker tw = t;
      get_parent_dir(tw);
      if(t->parent_dir == MYSTERY) throw hr_exception("unknown parent_dir (t) in add_road_shortcut");
      tpath.push_back(t->c.spin(t->parent_dir));
      t = t->move(t->parent_dir);
      }
    }
  }

EX int newcon;

EX void apply_road_shortcut(tcell *s) {
  if(flags & w_r3_no_road_shortcuts) return;
  auto& mem = qroad_memo[s];
  if(mem == qroad_for[s->id]) return;
  mem = qroad_for[s->id];
  shared_ptr<road_shortcut_trie_vertex> u;
  if(!road_shortcuts.count(s->id)) return;
  u = road_shortcuts[s->id];
  int q = tcellcount;
  while(true) {
    for(auto& v: u->backpaths) {
      auto s1 = s;
      for(auto x: v) {
        s1 = s1->cmove(x);
        be_solid(s1);
        }
      }
    twalker s0 = s; get_parent_dir(s0);
    if(!u->children.count(s->parent_dir)) break;
    u = u->children[s->parent_dir];
    s = s->move(s->parent_dir);
    }
  static int qmax = 0;
  newcon += tcellcount - q;
  if(tcellcount > q + qmax) println(hlog, "road shortcuts created ", qmax = tcellcount-q, " new connections");
  }

/** next roadsign ID -- they start at -100 and go downwards */
int next_roadsign_id = -100;

/** get the ID of a roadsign path */
EX map<vector<int>, int> roadsign_id;

EX int get_roadsign(twalker what) {
  int dlimit = what.at->dist - 1;
  tcell *s = what.at, *t = what.peek();
  apply_road_shortcut(s);
  vector<int> result;
  while(s->dist > dlimit) { 
    twalker s0 = s;
    get_parent_dir(s0);
    if(s->parent_dir == MYSTERY) throw hr_exception("parent_dir unknown");
    result.push_back(s->parent_dir); s = s->move(s->parent_dir);
    result.push_back(s->dist - dlimit);
    }
  vector<int> tail;
  while(t->dist > dlimit) {
    twalker t0 = t;
    get_parent_dir(t0);
    if(t->parent_dir == MYSTERY) throw hr_exception("parent_dir unknown");
    tail.push_back(t->dist - dlimit);
    tail.push_back(t->c.spin(t->parent_dir));
    t = t->move(t->parent_dir);
    }
  /* we reuse known_sides */
  vector<tcell*> vqueue;
  auto visit = [&] (tcell *c, int dir) {
    if(c->known_sides) return;
    c->known_sides = dir + 1;
    vqueue.push_back(c);
    };
  visit(s, MYSTERY);
  for(int i=0;; i++) {
    if(i == isize(vqueue)) throw hr_exception("vqueue empty");
    tcell *c = vqueue[i];
    if(c == t) break;
    for(int i=0; i<c->type; i++) {
      tcell *c1 = c->move(i);
      if(c1 && c1->dist <= dlimit)
        visit(c1, c->c.spin(i));
      if(c1 == t) break;
      }
    }
  while(t != s) {
    add_road_shortcut(s, t);
    int d = t->known_sides-1;
    tail.push_back(t->dist - dlimit);
    tail.push_back(t->c.spin(d));
    t = t->move(d);
    }
  for(auto c: vqueue) c->known_sides = 0;
  reverse(tail.begin(), tail.end());
  for(auto t: tail) result.push_back(t);
  if(roadsign_id.count(result)) return roadsign_id[result];
  return roadsign_id[result] = next_roadsign_id--;
  }

int last_qroad;

vector<vector<pair<int,int>>> possible_parents;

set<tcell*> imp_as_set;
int impcount;

struct vcell {
  int tid;
  vector<int> adj;
  void become(int _tid) { tid = _tid; adj.clear(); adj.resize(isize(treestates[tid].rules), -1); }
  };

struct vstate {
  bool need_cycle;
  vector<pair<int, int>> movestack;
  vector<vcell> vcells;
  vector<pair<int, pair<int, int>>> recursions;
  int current_pos;
  int current_root;
  vector<pair<int, int>> rpath;
  int steps;
  vstate() { steps = 0; }
  };

map<int, vector<int>> rev_roadsign_id;

int get_abs_rule(int tid, int j) {
  auto& ts = treestates[tid];
  int j1 = gmod(j - ts.giver.spin, isize(ts.rules));
  return ts.rules[j1];
  }

void be_important(tcell *c) {
  if(imp_as_set.count(c)) {
    return;
    }
  important.push_back(c);
  imp_as_set.insert(c);
  }

int error_debug = 0;

void build(vstate& vs, vector<tcell*>& places, int where, int where_last, tcell *g) {
  places[where] = g;
  twalker wh = g;
  auto ts0 = get_treestate_id(wh);
  if(error_debug >= 2) println(hlog, "[", where, "<-", where_last, "] [", g, " ] expected treestate = ", vs.vcells[where].tid, " actual treestate = ", ts0);

  vector<tcell*> v;
  vector<int> spins;
  for(int i=0; i<g->type; i++) {
    v.push_back(g->cmove(i));
    spins.push_back(g->c.spin(i));
    }
  if(error_debug >= 2) println(hlog, g, " -> ", v, " spins: ", spins);

  auto& c = vs.vcells[where];
  for(int i=0; i<isize(c.adj); i++)
    if(c.adj[i] != -1 && c.adj[i] != where_last) {
      indenter ind(2);
      int rule = get_abs_rule(vs.vcells[where].tid, i);
      auto g1 = g->cmove(i);
      twalker wh1 = g1;
      auto ts = get_treestate_id(wh1).second;
      if(ts != rule) {
        be_important(g);
        // be_important(treestates[ts0.second].giver.at);
        be_important(g1);
        // be_important(treestates[ts].giver.at);
        continue;
        }
      build(vs, places, c.adj[i], where, g1);
      }
  }

EX int max_ignore_level_pre = 3;
EX int max_ignore_level_post = 0;
EX int max_ignore_time_pre = 999999;
EX int max_ignore_time_post = 999999;
int ignore_level;

int check_debug = 0;

int side_errors, rpath_errors, dist_errors;

void error_found(vstate& vs) {
  if(error_debug >= 2) println(hlog, "current root = ", vs.current_root);
  int id = 0;
  for(auto& v: vs.vcells) {
    if(error_debug >= 2) println(hlog, "vcells[", id++, "]: tid=", v.tid, " adj = ", v.adj);
    }
  vector<tcell*> places(isize(vs.vcells), nullptr);
  tcell *g = treestates[vs.vcells[vs.current_root].tid].giver.at;
  int q = isize(important);
  build(vs, places, vs.current_root, -1, g);
  if(q == 0) for(auto& p: places) if(!p) throw rulegen_failure("bad tree");

  // for(auto p: places) be_important(p);
  // println(hlog, "added to important: ", places);
  for(auto rec: vs.recursions) {
    int at = rec.first;
    int dir = rec.second.first;
    int diff = rec.second.second;
    auto p = places[at];
    if(p) {
      auto p1 = p->cmove(dir);
      twalker pw = p;
      pw.at->code = MYSTERY_LARGE;
      int tsid = get_treestate_id(pw).second;
      if(!treestates[tsid].giver.at) {
        be_important(pw.at);
        continue;
        }
      if(error_debug >= 2 && imp_as_set.count(p) && imp_as_set.count(p1))
        println(hlog, "last: ", p, " -> ", p1, " actual diff = ", p1->dist, "-", p->dist, " expected diff = ", diff, " dir = ", dir, " ts = ", tsid);
      indenter ind(2);
      for(int i=0; i<pw.at->type; i++) {
        int r = get_abs_rule(tsid, i);
        if(error_debug >= 2) {
          if(r < 0 && r != DIR_PARENT) {
            println(hlog, "rule ", tie(tsid, i), " is: ", r, " which means ", rev_roadsign_id[r]);
            }
          else {
            println(hlog, "rule ", tie(tsid, i), " is: ", r);
            }
          }
        }
      int r = get_abs_rule(tsid, dir);
      if(error_debug >= 2 && r < 0 && r != DIR_PARENT) {
        tcell *px = p;
        auto rr = rev_roadsign_id[r];
        for(int i=0; i<isize(rr); i+=2) {
          px = px->cmove(rr[i]);
          println(hlog, " after step ", rr[i], " we get to ", px, " in distance ", px->dist);
          }
        println(hlog, "get_roadsign is ", get_roadsign(twalker(p, dir)));
        }
      // if(treestates[tsid].giver) be_important(treestates[tsid].giver.at);
      // println(hlog, "the giver of ", tsid, " is ", treestates[tsid].giver.at);
      be_important(p);
      be_important(p1);
      }
    }

  if(error_debug >= 1) println(hlog, "added to important ", isize(important)-q, " places, solid_errors = ", solid_errors, " distance warnings = ", distance_warnings);
  if(flags & w_r3_all_errors) return;

  if(isize(important) == impcount) {
    handle_distance_errors();
    throw rulegen_failure("nothing important added");
    }
  throw rulegen_retry("3D error subtree found");
  }

void check(vstate& vs) {
  dynamicval<int> vst(vs.steps, vs.steps + 1);
  if(vs.steps >= 5000) {
    println(hlog, "check does not seem to terminate: ", isize(vs.vcells), " cells, ", isize(vs.movestack), " stack");
    error_found(vs);
    throw rulegen_retry("check does not terminate");
    }

  if(check_debug >= 3) println(hlog, "vcells=", isize(vs.vcells), " pos=", vs.current_pos, " stack=", vs.movestack, " rpath=", vs.rpath);
  indenter ind(check_debug >= 3 ? 2 : 0);

  if(vs.movestack.empty()) {
    if(vs.need_cycle && vs.current_pos != 0) {
      if(error_debug >= 1) println(hlog, "rpath: ", vs.rpath, " does not cycle correctly");
      rpath_errors++;
      error_found(vs);
      return;
      }
    if(check_debug >= 2) println(hlog, "rpath: ", vs.rpath, " successful");
    return;
    }
  auto p = vs.movestack.back();
  auto& c = vs.vcells[vs.current_pos];

  int ctid = c.tid;
  int rule = get_abs_rule(ctid, p.first);

  /* connection already exists */
  if(c.adj[p.first] != -1) {
    int dif = (rule == DIR_PARENT) ? -1 : 1;
    if(p.second != dif && p.second != MYSTERY && !(flags & w_ignore_transducer_dist)) {
      if(error_debug >= 1)
        println(hlog, "error: connection ", p.first, " at ", vs.current_pos, " has distance ", dif, " but ", p.second, " is expected");
      dist_errors++;
      vs.recursions.push_back({vs.current_pos, p});
      error_found(vs);
      vs.recursions.pop_back();
      return;
      }
    dynamicval<int> d(vs.current_pos, c.adj[p.first]);
    vs.movestack.pop_back();
    check(vs);
    vs.movestack.push_back(p);
    }

  /* parent connection */
  else if(rule == DIR_PARENT) {

    if(isize(vs.rpath) >= ignore_level) {
      if(check_debug >= 1) println(hlog, "rpath: ", vs.rpath, " ignored for ", vs.movestack);
      return;
      }
    if(check_debug >= 3) println(hlog, "parent connection");

    dynamicval<int> r(vs.current_root, isize(vs.vcells));
    vs.vcells[vs.current_pos].adj[p.first] = vs.current_root;
    for(auto pp: possible_parents[ctid]) {
      if(check_debug >= 3) println(hlog, tie(vs.current_pos, p.first), " is a child of ", pp);
      vs.rpath.emplace_back(pp);
      vs.vcells.emplace_back();
      vs.vcells.back().become(pp.first);
      vs.vcells.back().adj[pp.second] = vs.current_pos;
      check(vs);
      vs.vcells.pop_back();
      vs.rpath.pop_back();
      }
    vs.vcells[vs.current_pos].adj[p.first] = -1;
    }

  /* child connection */
  else if(rule >= 0) {
    if(check_debug >= 3) println(hlog, "child connection");
    vs.vcells[vs.current_pos].adj[p.first] = isize(vs.vcells);
    vs.vcells.emplace_back();
    vs.vcells.back().become(rule);
    vs.vcells.back().adj[treestates[rule].giver.spin] = vs.current_pos;
    check(vs);
    vs.vcells.pop_back();
    vs.vcells[vs.current_pos].adj[p.first] = -1;
    }

  /* side connection */
  else {
    vs.recursions.push_back({vs.current_pos, p});
    auto& v = rev_roadsign_id[rule];
    if(v.back() != p.second + 1 && p.second != MYSTERY && !(flags & w_ignore_transducer_dist)) {
      if(error_debug >= 1) println(hlog, "error: side connection");
      side_errors++;
      error_found(vs);
      vs.recursions.pop_back();
      return;
      }
    int siz = isize(vs.movestack);
    vs.movestack.pop_back();
    if(check_debug >= 3) {
      println(hlog, "side connection: ", v);
      println(hlog, "entered recursions as ", vs.recursions.back(), " on position ", isize(vs.recursions)-1);
      }
    for(int i=v.size()-2; i>=0; i-=2) vs.movestack.emplace_back(v[i], i == 0 ? -1 : v[i+1] - v[i-1]);
    check(vs);
    vs.movestack.resize(siz);
    vs.movestack.back() = p;
    vs.recursions.pop_back();
    }
  }

bool check_det(vstate& vs) {
  indenter ind(check_debug >= 3 ? 2 : 0);
  back: ;
  if(check_debug >= 3) println(hlog, "vcells=", isize(vs.vcells), " pos=", vs.current_pos, " stack=", vs.movestack, " rpath=", vs.rpath);

  if(vs.movestack.empty()) {
    if(check_debug >= 2) println(hlog, "rpath: ", vs.rpath, " successful");
    return true;
    }
  auto p = vs.movestack.back();
  auto& c = vs.vcells[vs.current_pos];

  int ctid = c.tid;
  int rule = get_abs_rule(ctid, p.first);

  /* connection already exists */
  if(c.adj[p.first] != -1) {
    vs.current_pos = c.adj[p.first];
    int dif = (rule == DIR_PARENT) ? -1 : 1;
    if(p.second != dif && p.second != MYSTERY && !(flags & w_ignore_transducer_dist)) {
      error_found(vs);
      return false;
      }
    vs.movestack.pop_back();
    goto back;
    }

  /* parent connection */
  else if(rule == DIR_PARENT) {
    if((flags & w_r3_all_errors) && isize(important) > impcount) throw rulegen_retry("checking PARENT");
    throw rulegen_failure("checking PARENT");
    }

  /* child connection */
  else if(rule >= 0) {
    if(check_debug >= 3) println(hlog, "child connection");
    vs.vcells[vs.current_pos].adj[p.first] = isize(vs.vcells);
    vs.vcells.emplace_back();
    vs.vcells.back().become(rule);
    int dir = treestates[rule].giver.spin;
    vs.vcells.back().adj[dir] = vs.current_pos;
    goto back;
    }

  /* side connection */
  else {
    auto& v = rev_roadsign_id[rule];
    if(v.back() != p.second + 1 && p.second != MYSTERY && !(flags & w_ignore_transducer_dist)) {
      error_found(vs);
      return false;
      }
    vs.movestack.pop_back();
    if(check_debug >= 3) println(hlog, "side connection: ", v);
    for(int i=v.size()-2; i>=0; i-=2) vs.movestack.emplace_back(v[i], i == 0 ? -1 : v[i+1] - v[i-1]);
    goto back;
    }
  }

const int ENDED = -1;

struct transducer_state {
  int tstate1, tstate2;
  tcell *relation;
  bool operator < (const transducer_state& ts2) const { return tie(tstate1, tstate2, relation) < tie(ts2.tstate1, ts2.tstate2, ts2.relation); }
  bool operator == (const transducer_state& ts2) const { return tie(tstate1, tstate2, relation) == tie(ts2.tstate1, ts2.tstate2, ts2.relation); }
  };

struct transducer_transitions {
  flagtype accepting_directions;
  map<pair<int, int>, transducer_transitions*> t;
  transducer_transitions() { accepting_directions = 0; }
  };

inline void print(hstream& hs, transducer_transitions* h) { print(hs, "T", index_pointer(h)); }
inline void print(hstream& hs, const transducer_state& s) { print(hs, "S", tie(s.tstate1, s.tstate2, s.relation)); }

template<class T> size_t hsh(T t) { return std::hash<T>()(t); }

struct tshash {
  size_t operator() (const transducer_state& s) const {
    size_t res = hsh(s.tstate1) ^ (hsh(s.tstate2) << 16);
    res ^= size_t(s.relation) + 0x9e3779b9 + (res << 6) + (res >> 2);
    return res;
    }
  };

using tpair = pair<const transducer_state, transducer_transitions>;

using transducer = std::unordered_map<transducer_state, transducer_transitions, tshash>;

transducer autom;
int comp_step;

tcell* rev_move(tcell *t, int dir) {
  vector<int> dirs;
  while(t->dist) {
    twalker tw = t; get_parent_dir(tw);
    if(t->parent_dir == MYSTERY) {
      println(hlog, "dist = ", t->dist, " for ", t);
      throw rulegen_failure("no parent dir");
      }
    dirs.push_back(t->c.spin(t->parent_dir));
    t = t->move(t->parent_dir);
    }
  t->cmove(dir);
  dirs.push_back(t->c.spin(dir));
  t = t_origin[t->cmove(dir)->id].at;
  while(!dirs.empty()) {
    t = t->cmove(dirs.back());
    twalker tw = t; get_parent_dir(tw);
    if(t->dist && t->parent_dir == MYSTERY) throw rulegen_failure("no parent_dir assigned!");
    dirs.pop_back();
    }
  return t;
  }

tcell* get_move(tcell *c, int dir) {
  if(dir == ENDED) return c;
  return c->cmove(dir);
  }

struct tuplehash {
  size_t operator() (const tuple<tcell*, int, int>& tu) const { return hsh(get<0>(tu)) + (hsh(get<1>(tu)) << 8) + (hsh(get<2>(tu)) << 16); }
  };

std::unordered_map<tuple<tcell*, int, int>, tcell*, tuplehash> rmmemo;

tcell *rev_move2(tcell *t, int dir1, int dir2) {
  tuple<tcell*, int, int> key;
  get<0>(key) = t;
  get<1>(key) = dir1;
  get<2>(key) = dir2;
  auto& memo = rmmemo[key];
  if(memo) return memo;
  if(dir1 != ENDED) t = rev_move(t, dir1);
  if(dir2 != ENDED) {
    t = t->cmove(dir2);
    twalker tw = t; get_parent_dir(tw);
    if(t->dist && t->parent_dir == MYSTERY) throw rulegen_failure("no parent_dir assigned!");
    }
  twalker tw = t; get_parent_dir(tw);
  if(t->dist && t->parent_dir == MYSTERY) throw rulegen_failure("no parent_dir assigned after rev_move2!");
  return memo = t;
  }

vector<int> desc(tcell *t) {
  vector<int> dirs;
  while(t->dist) {
    if(t->parent_dir < 0) throw rulegen_failure("no parent dir");
    dirs.push_back(t->c.spin(t->parent_dir));
    t = t->move(t->parent_dir);
    }
  reverse(dirs.begin(), dirs.end());
  return dirs;
  }

template<class T> int build_vstate(vstate& vs, vector<int>& path1, const vector<int>& parent_dir, const vector<int>& parent_id, int at, T state) {
  vs.current_pos = vs.current_root = isize(vs.vcells);
  vs.vcells.emplace_back();
  vs.vcells.back().become(state(at));
  while(parent_id[at] != -1) {
    int ots = state(at);
    int dir = parent_dir[at];
    path1.push_back(dir);
    at = parent_id[at];
    if(dir == -1) continue;
    vs.vcells.emplace_back();
    vs.vcells.back().become(state(at));
    int xdir = treestates[ots].giver.at->parent_dir;
    vs.vcells[vs.current_root].adj[xdir] = vs.current_root+1;
    vs.vcells[vs.current_root+1].adj[dir] = vs.current_root;
    vs.current_root++;
    }
  reverse(path1.begin(), path1.end());
  return at;
  }

void gen_path(vstate &vs, vector<int>& path2) {
  while(vs.current_pos != vs.current_root) {
    auto g = treestates[vs.vcells[vs.current_pos].tid].giver;
    int dir = g.at->parent_dir;
    path2.push_back(g.at->c.spin(dir));
    vs.current_pos = vs.vcells[vs.current_pos].adj[dir];
    }
  reverse(path2.begin(), path2.end());
  }

int get_abs_rule1(int ts, int dir) {
  if(dir == ENDED) return ts;
  return get_abs_rule(ts, dir);
  }

void extract_identity(int tid, int ruleid, transducer& identity) {
  identity.clear();
  comp_step = 0;

  struct searcher { int ts; transducer_transitions *ires;
    bool operator < (const searcher& s2) const { return tie(ts, ires) < tie(s2.ts, s2.ires); }
    };

  set<searcher> in_queue;
  vector<searcher> q;
  auto enqueue = [&] (const searcher& s) {
    if(in_queue.count(s)) return;
    in_queue.insert(s);
    q.push_back(s);
    };

  for(auto t: t_origin) {
    transducer_state ts;
    ts.tstate1 = ts.tstate2 = get_treestate_id(t).second;
    ts.relation = t.at;
    searcher sch = searcher{ ts.tstate1, &(identity[ts]) };
    enqueue(sch);
    }

  for(int i=0; i<isize(q); i++) {
    auto sch = q[i];
    int dirs = isize(treestates[sch.ts].rules);
    bool ok = true;
    if(tid != -1 && treestates[sch.ts].giver.at->id != tid) ok = false;
    if(ruleid != -1 && ok) {
      ok = false;
      for(int d=0; d<dirs; d++) if(get_abs_rule(sch.ts, d) == ruleid) ok = true;
      }
    if(ok) sch.ires->accepting_directions = 1;
    for(int s=0; s<dirs; s++) {
      auto r = get_abs_rule(sch.ts, s);
      if(r < 0) continue;
      transducer_state ts;
      ts.tstate1 = ts.tstate2 = r;
      ts.relation = t_origin[treestates[r].giver.at->id].at;
      auto added = &(identity[ts]);
      sch.ires->t[{s, s}] = added;
      searcher next;
      next.ires = added;
      next.ts = r;
      enqueue(next);
      }
    }
  }

void compose_with(const transducer& tr, const transducer& dir, transducer& result) {
  println(hlog, "composing ", isize(tr), " x ", isize(dir));
  indenter ind(2);
  struct searcher {
    int ts1, ts2, ts3;
    bool fin1, fin2, fin3;
    tcell *tat;
    transducer_transitions *ires;
    const transducer_transitions *t1;
    const transducer_transitions *t2;
    bool operator == (const searcher& s2) const { return tie(fin1, fin2, fin3, ires, t1, t2) == tie(s2.fin1, s2.fin2, s2.fin3, s2.ires, s2.t1, s2.t2); };
    };

  struct searchhash {
    size_t operator() (const searcher& s) const {
      size_t res = size_t(s.fin1+2*s.fin2+4*s.fin3);
      res ^= size_t(s.ires) + 0x9e3779b9 + (res << 6) + (res >> 2);
      res ^= size_t(s.t1) + 0x9e3779b9 + (res << 6) + (res >> 2);
      res ^= size_t(s.t2) + 0x9e3779b9 + (res << 6) + (res >> 2);
      return res;
      }
    };

  std::unordered_set<searcher, searchhash> in_queue;
  vector<searcher> q;
  auto enqueue = [&] (const searcher& s) {
    if(in_queue.count(s)) return;
    in_queue.insert(s);
    q.push_back(s);
    };

  for(auto t: t_origin) {
    transducer_state ts;
    ts.tstate1 = ts.tstate2 = get_treestate_id(t).second;
    ts.relation = t.at;
    if(!tr.count(ts)) continue;
    if(!dir.count(ts)) continue;
    searcher sch = searcher{ ts.tstate1, ts.tstate1, ts.tstate1, false, false, false, t.at, &(result[ts]), &(tr.at(ts)), &(dir.at(ts)) };
    enqueue(sch);
    }
  
  int tdc = 0, tdc2 = 0;

  for(int i=0; i<isize(q); i++) {
    auto sch = q[i];
    if(sch.t1->accepting_directions && sch.t2->accepting_directions)
      sch.ires->accepting_directions = 1;

    searcher next;
    
    auto try_d3 = [&] (int d1, int d2, int r1, int r2) {
      tdc++;

      auto try_done = [&] (int d3, int r3) {
        tdc2++;
        next.ts1 = r1; next.fin1 = d1 == ENDED;
        next.ts2 = r2; next.fin2 = d2 == ENDED;
        next.ts3 = r3; next.fin3 = d3 == ENDED;
        next.tat = rev_move2(sch.tat, d1, d3);
        auto nstate_key = transducer_state { next.ts1, next.ts3, next.tat };

        next.ires = sch.ires;
        if(d1 != ENDED || d3 != ENDED)
          next.ires = sch.ires->t[{d1, d3}] = &(result[nstate_key]);

        enqueue(next);
        };
      
      if(d2 == ENDED) {
        next.t2 = sch.t2;
        try_done(ENDED, sch.ts3);
        }
      
      auto p = sch.t2->t.lower_bound({d2, -999});
      while(p != sch.t2->t.end() && p->first.first == d2) {
        int d3 = p->first.second;
        if(sch.fin3 && d3 != ENDED) break; // we can break right away
        auto r3 = get_abs_rule1(sch.ts3, d3);
        next.t2 = p->second;
        try_done(d3, r3);
        p++;
        }
      };
    
    next.t1 = sch.t1;
    try_d3(ENDED, ENDED, sch.ts1, sch.ts2);

    for(auto& p12: sch.t1->t) {
      int d1 = p12.first.first;
      int d2 = p12.first.second;
      if(sch.fin1 && d1 != ENDED) break; /* we can break right away -- no more to find */
      if(sch.fin2 && d2 != ENDED) continue; /* ... but here we cannot */
      auto r1 = get_abs_rule1(sch.ts1, d1);
      auto r2 = get_abs_rule1(sch.ts2, d2);

      next.t1 = p12.second;

      try_d3(d1, d2, r1, r2);
      }
    }

  println(hlog, "composition queue = ", isize(q), " tdc = ", tie(tdc, tdc2));
  }

void throw_identity_errors(const transducer& id, const vector<int>& cyc) {
  struct searcher {
    int ts;
    bool split;
    const transducer_transitions *at;
    bool operator < (const searcher& s2) const { return tie(ts, split, at) < tie(s2.ts, s2.split, s2.at); }
    };

  set<searcher> in_queue;
  vector<searcher> q;
  vector<int> parent_id;
  vector<int> parent_dir;
  auto enqueue = [&] (const searcher& s, int id, int dir) {
    if(in_queue.count(s)) return;
    in_queue.insert(s);
    q.push_back(s);
    parent_id.push_back(id);
    parent_dir.push_back(dir);
    };

  for(auto t: t_origin) {
    transducer_state ts;
    ts.tstate1 = ts.tstate2 = get_treestate_id(t).second;
    ts.relation = t.at;
    if(!id.count(ts)) continue;
    searcher sch = searcher{ ts.tstate1, false, &(id.at(ts)) };
    enqueue(sch, -1, -1);
    }

  for(int i=0; i<isize(q); i++) {
    auto sch = q[i];
    if(sch.at->accepting_directions && sch.split) {
      vstate vs;
      vs.need_cycle = true;
      for(auto v: cyc) vs.movestack.emplace_back(v, MYSTERY);
      vector<int> path1;
      build_vstate(vs, path1, parent_dir, parent_id, i, [&] (int i) { return q[i].ts; });
      println(hlog, "suspicious path found at ", path1);
      int err = vs.current_pos;
      bool ok = check_det(vs);
      if(ok) {
        vector<int> path2;
        gen_path(vs, path2);
        println(hlog, "after cycle: ", path2, " (", vs.current_pos, " vs ", err, ")");
        if(vs.current_pos != err) {
          rpath_errors++;
          error_found(vs);
          return;
          }
        else {
          if(isize(important) > impcount)
            throw rulegen_retry("suspicious path worked");
          else
            throw rulegen_failure("suspicious path worked");
          }
        }
      return;
      }
    for(auto p: sch.at->t) {
      int d = p.first.first;
      auto r = get_abs_rule1(sch.ts, d);
      if(r < 0) throw rulegen_failure("r<0");

      searcher next;
      next.ts = r;
      next.split = sch.split || p.first.first != p.first.second;
      next.at = p.second;

      enqueue(next, i, d);
      }
    }
  }

void throw_distance_errors(const transducer& id, int dir, int delta) {
  struct searcher {
    int ts;
    int diff;
    const transducer_transitions *at;
    bool operator < (const searcher& s2) const { return tie(ts, diff, at) < tie(s2.ts, s2.diff, s2.at); }
    };

  set<searcher> in_queue;
  vector<searcher> q;
  vector<int> parent_id;
  vector<int> parent_dir;
  auto enqueue = [&] (const searcher& s, int id, int dir) {
    if(in_queue.count(s)) return;
    in_queue.insert(s);
    q.push_back(s);
    parent_id.push_back(id);
    parent_dir.push_back(dir);
    };

  for(auto t: t_origin) {
    transducer_state ts;
    ts.tstate1 = ts.tstate2 = get_treestate_id(t).second;
    ts.relation = t.at;
    if(!id.count(ts)) continue;
    searcher sch = searcher{ ts.tstate1, false, &(id.at(ts)) };
    enqueue(sch, -1, -1);
    }

  for(int i=0; i<isize(q); i++) {
    auto sch = q[i];
    if(sch.at->accepting_directions && sch.diff != delta) {
      vstate vs;
      vs.need_cycle = true;
      vs.movestack = {{dir, MYSTERY}};
      vector<int> path1;
      build_vstate(vs, path1, parent_dir, parent_id, i, [&] (int i) { return q[i].ts; });
      println(hlog, "suspicious distance path found at ", path1);
      bool ok = check_det(vs);
      if(ok) throw rulegen_failure("suspicious distance path worked");
      return;
      }
    for(auto p: sch.at->t) {
      int d = p.first.first;
      auto r = get_abs_rule1(sch.ts, d);
      if(r < 0) throw rulegen_failure("r<0");

      searcher next;
      next.ts = r;
      next.diff = sch.diff - (p.first.first == ENDED ? 0:1) + (p.first.second == ENDED ? 0:1);
      next.at = p.second;

      enqueue(next, i, d);
      }
    }
  }

void extract(transducer& duc, transducer& res, int id, int dir) {
  map<transducer_transitions*, vector<transducer_transitions*>> edges;
  set<transducer_transitions*> productive;
  vector<transducer_transitions*> q;
  int acc = 0;
  for(auto& d: duc)
    for(auto edge: d.second.t)
      edges[edge.second].push_back(&d.second);
  auto enqueue = [&] (transducer_transitions* t) {
    if(productive.count(t)) return;
    productive.insert(t);
    q.push_back(t);
    };
  for(auto& d: duc)
    if(d.second.accepting_directions & (1<<dir))
      if(treestates[d.first.tstate1].giver.at->id == id)
        enqueue(&d.second), acc++;
  for(int i=0; i<isize(q); i++) {
    auto d = q[i];
    for(auto d2: edges[d])
      enqueue(d2);
    }
  println(hlog, "extract ", tie(id, dir), ": ", isize(duc), " -> ", isize(productive), " (acc = ", acc, ")");
  res.clear();
  map<transducer_transitions*, transducer_transitions*> xlat;
  for(auto& d: duc) if(productive.count(&d.second)) {
    xlat[&d.second] = &(res[d.first]);
    if(d.second.accepting_directions & (1<<dir))
      if(treestates[d.first.tstate1].giver.at->id == id)
        res[d.first].accepting_directions = 1;
    }
  for(auto &p: productive) {
    auto &r = xlat[p];
    for(auto rem: p->t) if(productive.count(rem.second)) r->t[rem.first] = xlat.at(rem.second);
    }
  }

void be_productive(transducer& duc) {
  map<transducer_transitions*, vector<transducer_transitions*>> edges;
  set<transducer_transitions*> productive;
  vector<transducer_transitions*> q;
  int acc = 0;
  for(auto& d: duc)
    for(auto edge: d.second.t)
      edges[edge.second].push_back(&d.second);
  auto enqueue = [&] (transducer_transitions* t) {
    if(productive.count(t)) return;
    productive.insert(t);
    q.push_back(t);
    };
  for(auto& d: duc)
    if(d.second.accepting_directions)
      enqueue(&d.second), acc++;
  for(int i=0; i<isize(q); i++) {
    auto d = q[i];
    for(auto d2: edges[d])
      enqueue(d2);
    }
  println(hlog, "productive: ", isize(duc), " -> ", isize(productive), " (acc = ", acc, ")");
  vector<transducer_state> unproductive;
  for(auto p: productive) {
    map<pair<int, int>, transducer_transitions*> remaining;
    for(auto rem: p->t) if(productive.count(rem.second)) remaining[rem.first] = rem.second;
    p->t = std::move(remaining);
    }
  for(auto& d: duc) if(productive.count(&d.second) == 0) unproductive.push_back(d.first);
  for(auto u: unproductive) duc.erase(u);
  }

EX void trace_relation(vector<int> path1, vector<int> path2, int id) {
  int trans = max(isize(path1), isize(path2));
  int ts1 = get_treestate_id(t_origin[id]).second;
  int ts2 = ts1;
  tcell *tat = t_origin[id].at;
  for(int i=0; i<trans; i++) {
    println(hlog, "states = ", tie(ts1, ts2), " relation = ", tat);
    int t1 = i < isize(path1) ? path1[i] : ENDED;
    int t2 = i < isize(path2) ? path2[i] : ENDED;
    tat = rev_move2(tat, t1, t2);
    ts1 = get_abs_rule1(ts1, t1);
    ts2 = get_abs_rule1(ts2, t2);
    println(hlog, "after moves: ", tie(t1, t2));
    }  
  println(hlog, "states = ", tie(ts1, ts2), " relation = ", tat);
  }

EX void make_path_important(tcell *s, vector<int> p) {
  for(auto i: p) if(i >= 0) {
    s = s->cmove(i);
    be_important(s);
    }
  }

EX void find_multiple_interpretation() {
  println(hlog, "looking for multiple_interpretations");
  struct searcher {
    int ts1, ts2, ts3;
    bool fin1, fin2, fin3;
    bool split;
    transducer_transitions *q2, *q3;
    bool operator < (const searcher& s2) const { return tie(ts1, ts2, ts3, fin1, fin2, fin3, split, q2, q3) < tie(s2.ts1, s2.ts2, s2.ts3, s2.fin1, s2.fin2, s2.fin3, s2.split, s2.q2, s2.q3); }
    };
  set<searcher> in_queue;
  vector<searcher> q;
  vector<int> parent_id, parent_dir1, parent_dir2, parent_dir3;

  auto enqueue = [&] (const searcher& sch, int pid, int pdir1, int pdir2, int pdir3) {
    if(in_queue.count(sch)) return;
    in_queue.insert(sch);
    q.emplace_back(sch);
    parent_id.emplace_back(pid);
    parent_dir1.emplace_back(pdir1);
    parent_dir2.emplace_back(pdir2);
    parent_dir3.emplace_back(pdir3);
    };

  for(auto t: t_origin) {
    transducer_state ts;
    ts.tstate1 = ts.tstate2 = get_treestate_id(t).second;
    ts.relation = t.at;
    searcher sch = searcher{ ts.tstate1, ts.tstate1, ts.tstate1, false, false, false, false, &(autom[ts]), &(autom[ts]) };
    enqueue(sch, -1, -1, -1, -1);
    }

  for(int i=0; i<isize(q); i++) {
    searcher sch = q[i];
    // println(hlog, i, ": ", tie(sch.ts1, sch.ts2, sch.ts3, sch.fin1, sch.fin2, sch.fin3, sch.split), tie(parent_id[i], parent_dir1[i], parent_dir2[i], parent_dir3[i]));

    flagtype both = sch.q2->accepting_directions & sch.q3->accepting_directions;
    if(both && !(sch.fin1 && sch.fin2 && sch.fin3) && sch.split) {
      int at = i;
      while(at >= 0) {
        auto& sch = q[at];
        println(hlog, at, ": ", tie(sch.ts1, sch.ts2, sch.ts3, sch.fin1, sch.fin2, sch.fin3, sch.split, sch.q2, sch.q3), tie(parent_id[at], parent_dir1[at], parent_dir2[at], parent_dir3[at]));       
        for(auto& r: autom) {
          if(&r.second == q[at].q2) println(hlog, "q2 relation is ", r.first.relation, ": ", desc(r.first.relation));
          if(&r.second == q[at].q3) println(hlog, "q3 relation is ", r.first.relation, ": ", desc(r.first.relation));
          }
        at = parent_id[at];
        }
      vector<int> path1, path2, path3, path4;
      int xdir = -1;
      for(int dir=0; dir<64; dir++) if(both & (1ll<<dir)) xdir = dir;
      println(hlog, "multiple interpretation found for xdir = ", xdir);
      vstate vs;
      at = build_vstate(vs, path1, parent_dir1, parent_id, i, [&] (int i) { return q[i].ts1; });
      int at0 = at;
      println(hlog, path1);
      vs.movestack = {{xdir, MYSTERY}};
      bool ok = check_det(vs);
      if(!ok) return;
      gen_path(vs, path4);
      println(hlog, "path4 = ", path4);
      build_vstate(vs, path2, parent_dir2, parent_id, i, [&] (int i) { return q[i].ts2; });
      println(hlog, path2);
      build_vstate(vs, path3, parent_dir3, parent_id, i, [&] (int i) { return q[i].ts3; });
      println(hlog, path3);
      tcell *s = treestates[q[at].ts1].giver.at;
      auto s1=s, s2=s, s3=s;
      for(auto p: path1) s1 = rev_move2(s1, ENDED, p);
      for(auto p: path2) s2 = rev_move2(s2, ENDED, p);
      for(auto p: path3) s3 = rev_move2(s3, ENDED, p);
      println(hlog, "reached: ", tie(s1, s2, s3), " should reach: ", s1->cmove(xdir));
      trace_relation(path1, path2, treestates[q[at0].ts1].giver.at->id);
      trace_relation(path1, path3, treestates[q[at0].ts1].giver.at->id);
      make_path_important(s1, path1);
      make_path_important(s2, path1);
      make_path_important(s3, path1);
      if(isize(important) == impcount) throw rulegen_failure("nothing important added");
      if(!(flags & w_r3_all_errors)) throw rulegen_retry("multiple interpretation");
      }

    int dirs1 = isize(treestates[sch.ts1].rules);
    int dirs2 = isize(treestates[sch.ts2].rules);
    int dirs3 = isize(treestates[sch.ts3].rules);

    for(int dir1=ENDED; dir1<dirs1; dir1++)
    for(int dir2=ENDED; dir2<dirs2; dir2++)
    for(int dir3=ENDED; dir3<dirs3; dir3++) {
      if(dir1 >= 0 && sch.fin1) continue;
      if(dir2 >= 0 && sch.fin2) continue;
      if(dir3 >= 0 && sch.fin3) continue;
      searcher next;
      next.ts1 = get_abs_rule(sch.ts1, dir1);
      if(next.ts1 < 0) continue;
      next.ts2 = get_abs_rule(sch.ts2, dir2);
      if(next.ts2 < 0) continue;
      next.ts3 = get_abs_rule(sch.ts3, dir3);
      if(next.ts3 < 0) continue;
      if(!sch.q2->t.count({dir1, dir2})) continue;
      if(!sch.q3->t.count({dir1, dir3})) continue;
      next.q2 = sch.q2->t[{dir1, dir2}];
      next.q3 = sch.q3->t[{dir1, dir3}];
      next.fin1 = dir1 == ENDED;
      next.fin2 = dir2 == ENDED;
      next.fin3 = dir3 == ENDED;
      next.split = sch.split || (dir2 != dir3);
      enqueue(next, i, dir1, dir2, dir3);
      }
    }

  if((flags & w_r3_all_errors) && isize(important) > impcount) throw rulegen_retry("no multiple interpretation found after importants added");
  throw rulegen_failure("no multiple interpretation found");
  }

EX int max_err_iter = 4;

EX void test_transducers() {
  if(flags & w_skip_transducers) return;
  autom.clear();
  int iterations = 0;
  int multiple_interpretations = 0;
  int err_iter = 0;
  while(true) {
    next_iteration:
    check_timeout();
    iterations++;
    int changes = 0;
    multiple_interpretations = 0;

    struct searcher {
      int ts;
      vector<transducer_transitions*> pstates;
      // bool operator < (const searcher& s2) const { return tie(ts, pstates) < tie(s2.ts, s2.pstates); }
      bool operator == (const searcher& s2) const { return tie(ts, pstates) == tie(s2.ts, s2.pstates); }
      };

    struct searchhash {
      size_t operator() (const searcher& s) const {
        size_t res = hsh(s.ts);
        // https://stackoverflow.com/questions/20511347/a-good-hash-function-for-a-vector
        for(auto p: s.pstates) res ^= size_t(p) + 0x9e3779b9 + (res << 6) + (res >> 2);
        return res;
        }
      };

    std::unordered_set<searcher, searchhash> in_queue;
    vector<searcher> q;
    vector<int> parent_id;
    vector<int> parent_dir;

    auto enqueue = [&] (const searcher& sch, int pid, int pdir) {
      if(in_queue.count(sch)) return;
      in_queue.insert(sch);
      q.emplace_back(sch);
      parent_id.emplace_back(pid);
      parent_dir.emplace_back(pdir);
      };

    for(auto t: t_origin) {
      transducer_state ts;
      ts.tstate1 = ts.tstate2 = get_treestate_id(t).second;
      ts.relation = t.at;
      searcher sch = searcher{ ts.tstate1, { &(autom[ts]) } };
      enqueue(sch, -1, -1);
      }

    auto process = [&] (int i) {
      searcher sch = q[i];

      int dirs = isize(treestates[sch.ts].rules);
      // println(hlog, i, ". ", "ts ", sch.ts, " states=", isize(sch.pstates), " from = ", tie(q[i].parent_dir, q[i].parent_dir));

      for(int dir=0; dir<dirs; dir++) {
        int qty = 0;
        for(auto v: sch.pstates) if(v->accepting_directions & (1<<dir)) qty++;
        for(auto v: sch.pstates) for(auto& p: v->t) if(p.first.first == ENDED && (p.second->accepting_directions & (1<<dir))) qty++;
        if(qty > 1) {
          multiple_interpretations++;
          // print_transducer(autom);
          if(!(flags & w_r3_all_errors)) {
            vstate vs;
            vector<int> path1;
            int at = build_vstate(vs, path1, parent_dir, parent_id, i, [&] (int i) { return q[i].ts; });
            println(hlog, "after path = ", path1, " got multiple interpretation");
            for(auto v: sch.pstates) if(v->accepting_directions & (1<<dir)) println(hlog, "state ", v);
            for(auto v: sch.pstates) for(auto& p: v->t) if(p.first.first == ENDED && (p.second->accepting_directions & (1<<dir))) println(hlog, "state ", v, " after accepting END/", p.first.second);
            println(hlog, "starting at state: ", q[at].ts, " reached state ", q[i].ts);

            at = i;
            while(true) {
              println(hlog, "state ", q[at].ts, " vs ", q[at].pstates, " dir = ", parent_dir[at]);
              at = parent_id[at];
              if(at == -1) break;
              }

            find_multiple_interpretation();
            }
          }
        if(qty == 0) {
          vstate vs;
          vs.need_cycle = false;
          vs.movestack = { { dir, MYSTERY } };
          vector<int> path1, path2;
          int at = build_vstate(vs, path1, parent_dir, parent_id, i, [&] (int j) { return q[j].ts; });
          bool ok = check_det(vs);
          if(!ok) return;
          gen_path(vs, path2);
          int trans = max(isize(path1), isize(path2));
          int ts1 = q[at].ts;
          int ts2 = q[at].ts;
          tcell *tat = treestates[ts1].giver.at;
          // println(hlog, "root ", tat->id, " connecting ", path1, " dir ", dir, " to ", path2);
          auto cstate = q[at].pstates[0];
          auto cstate_key = transducer_state {ts1, ts1, tat };
          for(int i=0; i<trans; i++) {
            int t1 = i < isize(path1) ? path1[i] : ENDED;
            int t2 = i < isize(path2) ? path2[i] : ENDED;
            tat = rev_move2(tat, t1, t2);
            ts1 = get_abs_rule1(ts1, t1);
            ts2 = get_abs_rule1(ts2, t2);
            auto nstate_key = transducer_state {ts1, ts2, tat};
            auto nstate = &(autom[nstate_key]);
            if(cstate->t[{t1, t2}] && cstate->t[{t1,t2}] != nstate) {
              println(hlog, "conflict!");
              exit(1);
              }
            // println(hlog, cstate, " at ", cstate_key, " gets ", nstate, " at ", nstate_key, " in direction ", tie(t1, t2)); 
            cstate->t[{t1, t2}] = nstate;
            cstate = nstate;
            cstate_key = nstate_key;
            }
          cstate->accepting_directions |= (1<<dir);
          changes++;
          // goto next_iteration;
          }
        }

      /* all OK here */
      for(int s=0; s<dirs; s++) {
        auto r = get_abs_rule(sch.ts, s);
        if(r < 0) continue;
        searcher next;
        next.ts = r;
        for(auto v: sch.pstates) for(auto& p: v->t) if(p.first.first == s) next.pstates.push_back(p.second);
        sort(next.pstates.begin(), next.pstates.end());
        auto ip = std::unique(next.pstates.begin(), next.pstates.end());
        next.pstates.resize(ip - next.pstates.begin());
        enqueue(next, i, s);
        }
      };

    for(int i=0; i<isize(q); i++) process(i);
    
    if(changes) {
      println(hlog, "iteration ", iterations, ": changes = ", changes, " with ", isize(autom), " states, queue size = ", isize(q), ", add = ", isize(important)-impcount, " MI = ", multiple_interpretations);
      if(isize(important) > impcount || multiple_interpretations) err_iter++;
      if(err_iter < max_err_iter) goto next_iteration;
      if(err_iter == max_err_iter) max_err_iter++;
      }

    if((flags & w_r3_all_errors) && isize(important) > impcount) throw rulegen_retry("errors found by transducers");

    if((flags & w_r3_all_errors) && multiple_interpretations) {
      println(hlog, "multiple interpretations reported: ", multiple_interpretations);
      find_multiple_interpretation();
      }

    println(hlog, "transducers found successfully after ", iterations, " iterations, ", isize(autom), " states checked, queue size = ", isize(q), " rmmemo size = ", isize(rmmemo));
    
    vector<vector<transducer>> special(isize(t_origin));
    for(int tid=0; tid<isize(t_origin); tid++) {
      int dirs = t_origin[tid].at->type;
      special[tid].resize(dirs);
      for(int dir=0; dir<dirs; dir++)
        extract(autom, special[tid][dir], tid, dir);
      }

    set<cycle> checked;

    if(!(flags & w_skip_transducer_loops)) for(int tid=0; tid<isize(t_origin); tid++) {
      int id = 0;

      /* if correct, each loop iteration recovers the identity, so we can build it just once */
      transducer cum;
      extract_identity(tid, -1, cum);
      be_productive(cum);
      int id_size = isize(cum);

      for(auto& cyc: cycle_data[tid]) {
        if(checked.count(cyc)) {
          continue;
          }
        auto cyc2 = cyc;
        int q = isize(cyc.dirs);
        for(int i=0; i<q; i++) {
          for(int j=1; j<q; j++) {
            swap(cyc2.dirs[j], cyc2.dirs[j-1]);
            swap(cyc2.tids[j], cyc2.tids[j-1]);
            swap(cyc2.rdirs[j], cyc2.rdirs[j-1]);
            }
          checked.insert(cyc2);
          }
        println(hlog, "Working on tid=", tid, " cycle ", cyc.dirs, " (", id++, "/", isize(cycle_data[tid]), ")");
        check_timeout();
        indenter ind(2);
        int ctid = tid;
        for(auto c: cyc.dirs) {
          transducer result;
          println(hlog, "special is ", tie(ctid, c));
          compose_with(cum, special[ctid][c], result);
          be_productive(result);
          swap(cum, result);
          ctid = t_origin[ctid].at->cmove(c)->id;
          }
        throw_identity_errors(cum, cyc.dirs);
        if(id_size != isize(cum)) println(hlog, "error: identity not recovered correctly");
        }
      }
    if((flags & w_r3_all_errors) && isize(important) > impcount) throw rulegen_retry("loop errors found by transducers");

    if(!(flags & w_skip_transducer_terminate)) {
      println(hlog, "Verifying distances");
      
      map<pair<int, int>, vector< pair<int, int>> > by_roadsign;

      for(int tsid=0; tsid<isize(treestates); tsid++) 
      for(int dir=0; dir<treestates[tsid].giver.at->type; dir++) {
        int r = get_abs_rule(tsid, dir);
        if(r >= 0 || r == DIR_PARENT) continue;
        by_roadsign[{treestates[tsid].giver.at->id, r}].emplace_back(tsid, dir);
        }
      
      int id = 0;
      for(auto& p: by_roadsign) {
        int ctid = p.first.first;
        int r = p.first.second;
        auto& v = rev_roadsign_id.at(r);
        println(hlog, "Working on rule ", v, " at ", ctid, " (#", id++, "/", isize(by_roadsign), "), found in ", p.second);
        check_timeout();
        indenter ind(2);
        transducer cum;
        extract_identity(-1, r, cum);
        be_productive(cum);
        if(cum.empty()) { println(hlog, "does not exist!"); continue; }
        for(int i=0; i<isize(v); i+=2) {
          int c = v[i];
          transducer result;
          println(hlog, "special is ", tie(ctid, c));
          compose_with(cum, special[ctid][c], result);
          be_productive(result);
          swap(cum, result);
          println(hlog, "should be ", v[i+1] - 1);
          throw_distance_errors(cum, p.second[0].second, v[i+1] - 1);
          ctid = t_origin[ctid].at->cmove(c)->id;
          }
        }
      }
    if((flags & w_r3_all_errors) && isize(important) > impcount) throw rulegen_retry("rule distance errors found by transducers");
    break;
    }
  }

EX void check_upto(int lev, int t) {
  vstate vs;
  int N = isize(treestates);
  Uint32 start = SDL_GetTicks();
  for(ignore_level=1; ignore_level <= lev; ignore_level++) {
    println(hlog, "test ignore_level ", ignore_level);
    vs.need_cycle = false;
    side_errors = rpath_errors = dist_errors = 0;

    for(int i=0; i<N; i++) {
      for(int j=0; j<isize(treestates[i].rules); j++) {
        if(SDL_GetTicks() > start + t) return;
        check_timeout();
        int r = get_abs_rule(i, j);
        if(r < 0 && r != DIR_PARENT) {
          vs.vcells.clear();
          vs.vcells.resize(1);
          vs.vcells[0].become(i);
          vs.current_pos = vs.current_root = 0;
          vs.movestack = { {j, MYSTERY} };
          if(check_debug >= 1) println(hlog, "checking ", tie(i, j));
          indenter ind(2);
          check(vs);
          }
        }
      }

    vs.need_cycle = true;
    for(int i=0; i<N; i++) {
      int id = treestates[i].giver.at->id;
      for(auto &cd: cycle_data[id]) {
        if(SDL_GetTicks() > start + t) return;
        check_timeout();
        vs.vcells.clear();
        vs.vcells.resize(1);
        vs.vcells[0].become(i);
        vs.current_pos = vs.current_root = 0;
        vs.movestack.clear();
        for(auto v: cd.dirs) vs.movestack.emplace_back(v, MYSTERY);
        reverse(vs.movestack.begin(), vs.movestack.end());
        if(check_debug >= 1) println(hlog, "checking ", tie(i, id, cd.dirs));
        indenter ind(2);
        check(vs);
        }
      }

    if((flags & w_r3_all_errors) && isize(important) > impcount) {
      println(hlog, "found errors: side ", side_errors, " dist ", dist_errors, " rpath ", rpath_errors);
      throw rulegen_retry("errors found");
      }
    }
  }

EX void optimize() {

  imp_as_set.clear();
  for(auto t: important) imp_as_set.insert(t.at);

  /* optimize givers */
  set<int> seen;
  vector<int> oqueue;

  int changes = 0;
  int errors = 0;

  auto visit = [&] (twalker w, int expected, twalker parent) {
    int id = get_treestate_id(w).second;
    if(expected >= 0 && expected != id) {
      errors++;
      important.push_back(parent);
      important.push_back(w);
      return;
      }
    if(seen.count(id)) return;
    seen.insert(id);
    oqueue.push_back(id);
    auto& g = treestates[id].giver;
    if(g != w) changes++;
    g = w;
    };

  for(auto t: t_origin) visit(t, -1, t);

  for(int ii=0; ii<isize(oqueue); ii++) {
    int i = oqueue[ii];
    for(int j=0; j<isize(treestates[i].rules); j++) if(treestates[i].rules[j] >= 0)
      visit(treestates[i].giver + j + wstep, treestates[i].rules[j], treestates[i].giver);
    }

  int N = isize(treestates);

  if(rdebug_flags & 64) println(hlog, "optimize: changes = ", changes, " errors = ", errors, " unreachable = ", N - isize(seen));

  if(errors) throw rulegen_retry("error found in optimize");

  int steps = 0;
  for(int i=0; i<N; i++) if(!seen.count(i)) {
    twalker at = treestates[i].giver;
    if(!at.at) continue;
    int r = i;
    while(true) {
      if(at.at->dist == 0) throw rulegen_failure("reached the root");
      steps++;
      get_parent_dir(at);
      if(at.at->parent_dir == -1) throw rulegen_failure("no parent_dir for at");
      at.spin = at.at->parent_dir;
      at += wstep;
      get_parent_dir(at);
      if(at.at->parent_dir == -1) throw rulegen_failure("no parent_dir for at2");
      int r2 = get_treestate_id(at).second;
      auto at2 = at;
      at2.spin = at.at->parent_dir;
      if(at.at->dist == 0) at.at->parent_dir = 0;
      int j = -1;
      for(int k=0; k<at.at->type; k++) if(at2 + k == at) j = k;
      if(treestates[r2].rules.empty()) {
        important.push_back(at);
        break;
        }

      // println(hlog, "found: ", r2, " seen: ", int(seen.count(r2)), " expected: ", r, " found: ", treestates[r2].rules[j], " dist=", at.at->dist);

      if(treestates[r2].rules[j] != r) {
        // println(hlog, "expected: ", r, " found: ", treestates[r2].rules[j], " add ", at+wstep, at2);
        if(imp_as_set.count((at+wstep).at) && imp_as_set.count(at2.at))
          throw rulegen_failure("already in imp");
        important.push_back(at+wstep); important.push_back(at2); break;
        }
      r = r2; steps++;
      }
    }

  if(steps) { if(rdebug_flags & 64) println(hlog, "steps = ", steps); throw rulegen_retry("unreachable found in optimize"); }

  important.clear();
  for(auto s: seen) important.push_back(treestates[s].giver);
  }

EX void check_road_shortcuts() {
  println(hlog, "road shortcuts = ", qroad, " treestates = ", isize(treestates), " roadsigns = ", next_roadsign_id, " tcellcount = ", tcellcount, " try = ", try_count);
  if(qroad > last_qroad) {
    println(hlog, "qroad_for = ", qroad_for);
    println(hlog, "newcon = ", newcon, " tcellcount = ", tcellcount); newcon = 0;
    clear_codes();
    last_qroad = qroad;
    roadsign_id.clear();
    next_roadsign_id = -100;
    throw rulegen_retry("new road shortcuts");
    }
  }

EX void check_validity_3d() {
  println(hlog, "checking validity, important = ", isize(important));
  imp_as_set.clear();
  for(auto t: important) imp_as_set.insert(t.at);
  impcount = isize(important);
  possible_parents.clear();
  int N = isize(treestates);
  possible_parents.resize(N);
  for(int i=0; i<N; i++) {
    auto& ts = treestates[i];
    for(int j=0; j<isize(ts.rules); j++) if(ts.rules[j] >= 0)
      possible_parents[ts.rules[j]].emplace_back(i, gmod(j + ts.giver.spin, isize(ts.rules)));
    }

  rev_roadsign_id.clear();
  for(auto& rs: roadsign_id) rev_roadsign_id[rs.second] = rs.first;

  check_upto(max_ignore_level_pre, max_ignore_time_pre);
  test_transducers();
  check_upto(max_ignore_level_post, max_ignore_time_post);

  println(hlog, "Got it!");
  }

#if HDR
struct cycle {
  vector<int> dirs;
  vector<int> tids;
  vector<int> rdirs;
  bool operator < (const cycle& c2) const { return tie(dirs, tids, rdirs) < tie(c2.dirs, c2.tids, c2.rdirs); }
  };
#endif

EX vector<vector<cycle>> cycle_data;

EX void build_cycle_data() {
  cycle_data.clear();
  cycle_data.resize(number_of_types());
  for(int t=0; t<number_of_types(); t++) {
    cell *start = tcell_to_cell[t_origin[t].at];
    auto& sh0 = currentmap->get_cellshape(start);
    for(int i=0; i<start->type; i++) {
      auto& f = sh0.faces[i];
      for(int j=0; j<isize(f); j++) {
        hyperpoint v1 = kleinize(sh0.from_cellcenter * sh0.faces[i][j]);
        hyperpoint v2 = kleinize(sh0.from_cellcenter * sh0.faces[i][(j+1) % isize(f)]);
        cycle cc;
        cc.dirs = {i};
        cc.tids = {t};
        start->cmove(i);
        cc.rdirs = {start->c.spin(i)};
        transmatrix T = currentmap->adj(start, i);
        cell *at = start->cmove(i);
        cell *last = start;
        while(at != start) {
          auto &sh1 = currentmap->get_cellshape(at);
          int dir = -1;
          for(int d=0; d<at->type; d++) if(at->cmove(d) != last) {
            int ok = 0;
            for(auto rv: sh1.faces[d]) {
              hyperpoint v = kleinize(T * sh1.from_cellcenter * rv);
              if(sqhypot_d(3, v-v1) < 1e-6) ok |= 1;
              if(sqhypot_d(3, v-v2) < 1e-6) ok |= 2;
              }
            if(ok == 3) dir = d;
            }
          if(dir == -1) throw hr_exception("cannot cycle");
          cc.tids.push_back(get_id(at));
          cc.dirs.push_back(dir);
          cc.rdirs.push_back(at->c.spin(dir));
          T = T * currentmap->adj(at, dir);
          last = at;
          at = at->cmove(dir);
          }
        cycle_data[t].push_back({std::move(cc)});
        }
      }
    }
  println(hlog, "cycle data computed");
  }

using classdata = pair<vector<int>, int>;
vector<classdata> nclassify;
vector<int> representative;

void genhoneycomb(string fname) {
  if(WDIM != 3) throw hr_exception("genhoneycomb not in honeycomb");
  if(!known()) throw hr_exception("rules not known");

  int qc = isize(t_origin);

  vector<short> data;
  string side_data;

  map<int, vector<int>> rev_roadsign_id;
  for(auto& rs: roadsign_id) rev_roadsign_id[rs.second] = rs.first;

  int N = isize(treestates);
  nclassify.clear();
  nclassify.resize(N);
  for(int i=0; i<N; i++) nclassify[i] = {{0}, i};

  int numclass = 1;
  while(true) {
    println(hlog, "N = ", N, " numclass = ", numclass);
    for(int i=0; i<N; i++) {
      auto& ts = treestates[i];
      for(int j=0; j<isize(ts.rules); j++) {
        int j1 = gmod(j - ts.giver.spin, isize(ts.rules));
        auto r = ts.rules[j1];
        if(r < 0) nclassify[i].first.push_back(r);
        else nclassify[i].first.push_back(nclassify[r].first[0]);
        }
      }
    sort(nclassify.begin(), nclassify.end());
    vector<int> last = {}; int newclass = 0;
    for(int i=0; i<N; i++) {
      if(nclassify[i].first != last) {
        newclass++;
        last = nclassify[i].first;
        }
      nclassify[i].first = {newclass-1};
      }
    sort(nclassify.begin(), nclassify.end(), [] (const classdata& a, const classdata& b) { return a.second < b.second; });
    if(numclass == newclass) break;
    numclass = newclass;
    }
  representative.resize(numclass);
  for(int i=0; i<isize(treestates); i++) representative[nclassify[i].first[0]] = i;

  println(hlog, "Minimized rules (", numclass, " states):");
  for(int i=0; i<numclass; i++) {
    auto& ts = treestates[representative[i]];
    print(hlog, lalign(4, i), ":");
    for(int j=0; j<isize(ts.rules); j++) {
      int j1 = gmod(j - ts.giver.spin, isize(ts.rules));
      auto r =ts.rules[j1];
      if(r == DIR_PARENT) print(hlog, " P");
      else if(r >= 0) print(hlog, " ", nclassify[r].first[0]);
      else print(hlog, " S", r);
      }
    println(hlog);
    }
  println(hlog);

  vector<int> childpos;

  for(int i=0; i<numclass; i++) {
    childpos.push_back(isize(data));
    auto& ts = treestates[representative[i]];
    for(int j=0; j<isize(ts.rules); j++) {
      int j1 = gmod(j - ts.giver.spin, isize(ts.rules));
      auto r =ts.rules[j1];
      if(r == DIR_PARENT) {
        data.push_back(-1);
        side_data += ('A' + j);
        side_data += ",";
        }
      else if(r >= 0) {
        data.push_back(nclassify[r].first[0]);
        }
      else {
        data.push_back(-1);
        auto& str = rev_roadsign_id[r];
        bool next = true;
        for(auto ch: str) {
          if(next) side_data += ('a' + ch);
          next = !next;
          }
        side_data += ",";
        }
      }
    }
  childpos.push_back(isize(data));

  shstream ss;

  ss.write(ss.get_vernum());
  mapstream::save_geometry(ss);
  ss.write(fieldpattern::use_rule_fp);
  ss.write(fieldpattern::use_quotient_fp);
  ss.write(reg3::minimize_quotient_maps);

  auto& fp = currfp;
  hwrite_fpattern(ss, fp);

  vector<int> root(qc, 0);
  for(int i=0; i<qc; i++) root[i] = nclassify[get_treestate_id(t_origin[i]).second].first[0];
  println(hlog, "root = ", root);
  hwrite(ss, root);

  println(hlog, "data = ", data);
  hwrite(ss, data);
  println(hlog, "side_data = ", side_data);
  hwrite(ss, side_data);
  println(hlog, "childpos = ", childpos);
  hwrite(ss, childpos);

  println(hlog, "compress_string");
  string s = compress_string(ss.s);

  fhstream of(fname, "wb");
  print(of, s);
  }

EX void cleanup3() {
  roadsign_id.clear();
  rev_roadsign_id.clear();
  next_roadsign_id = -100;
  autom.clear();
  cycle_data.clear();
  road_shortcuts.clear();
  qroad_for.clear();
  qroad_memo.clear();
  possible_parents.clear();
  rmmemo.clear();
  max_err_iter = 4;
  }

#if CAP_COMMANDLINE
int readRuleArgs3() {
  using namespace arg;
  if(0) ;
  else if(argis("-gen-honeycomb")) {
    shift(); genhoneycomb(arg::args());
    }

  else if(argis("-urq")) {
    // -urq 7 to prepare honeycomb generation
    stop_game();
    shift(); int i = argi();
    reg3::consider_rules = i >> 3;
    fieldpattern::use_rule_fp = (i & 1) ? 1 : 0;
    fieldpattern::use_quotient_fp = (i & 2) ? 1 : 0;
    reg3::minimize_quotient_maps = (i & 4) ? 1 : 0;
    }

  else if(argis("-load-honeycomb")) {
    PHASE(3);
    stop_game();
    shift(); string s = arg::args();
    reg3::replace_rule_file = s;
    shstream ins(decompress_string(read_file_as_string(s)));
    ins.read(ins.vernum);
    mapstream::load_geometry(ins);
    reg3::consider_rules = 2;
    }

  else if(argis("-less-states")) {
    shift(); rulegen::less_states = argi();
    }

  else if(argis("-dump-rules")) {
    start_game();
    shift(); reg3::dump_rules(arg::args());
    }

  else if(argis("-clean-rules")) {
    cleanup();
    }

  else return 1;
  return 0;
  }

auto hook3 = addHook(hooks_args, 100, readRuleArgs3);
#endif

EX }
#endif

}
