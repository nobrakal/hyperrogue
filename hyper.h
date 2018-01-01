// scale the Euclidean
#define EUCSCALE 2.3

#define NUMWITCH 7

// achievements

#define LB_YENDOR_CHALLENGE 40
#define LB_PURE_TACTICS 41
#define NUMLEADER 72
#define LB_PURE_TACTICS_SHMUP 49
#define LB_PURE_TACTICS_COOP 50

extern int currentscore[NUMLEADER];
extern int syncstate;

// initialize the achievement system.
void achievement_init();

// close the achievement system.
void achievement_close();

// gain the achievement with the given name.
// flags: 'e' - for Euclidean, 's' - for Shmup, '7' - for heptagonal
// Only awarded if special modes are matched exactly.
void achievement_gain(const char*, char flags = 0);

// gain the achievement for collecting a number of 'it'.
void achievement_collection(eItem it, int prevgold, int newgold);

// this is used for 'counting' achievements, such as kill 10
// monsters at the same time.
void achievement_count(const string& s, int current, int prev);

// scores for special challenges
void achievement_score(int cat, int score);

// gain the victory achievements. Set 'hyper' to true for
// the Hyperstone victory, and false for the Orb of Yendor victory.
void achievement_victory(bool hyper);

// gain the final achievements. Called with really=false whenever the user
// looks at their score, and really=true when the game really ends.
void achievement_final(bool really);

// display the last achievement gained.
void achievement_display();

// call the achievement callbacks
void achievement_pump();

// achievements received this game
vector<string> achievementsReceived;

// game forward declarations
typedef int flagtype;

bool mirrorkill(cell *c);
bool isNeighbor(cell *c1, cell *c2);
void checkTide(cell *c);
namespace anticheat { extern bool tampered; }
int numplayers();
void removeIvy(cell *c);
bool cellEdgeUnstable(cell *c, flagtype flags = 0);
int coastvalEdge(cell *c);
typedef int cellfunction(cell*);
int towerval(cell *c, cellfunction* cf = &coastvalEdge);
#define HRANDMAX 0x7FFFFFFF
int hrandpos(); // 0 to HRANDMAX
void restartGame(char switchWhat = 0, bool push = false);
int landMultiplier(eLand l);
eItem treasureType(eLand l);
void buildBarrier(cell *c, int d, eLand l = laNone);
void extendBarrier(cell *c);
bool buildBarrier4(cell *c, int d, int mode, eLand ll, eLand lr);
bool buildBarrier6(struct cellwalker cw, int type);
bool makeEmpty(cell *c);
bool isCrossroads(eLand l);
enum orbAction { roMouse, roKeyboard, roCheck, roMouseForce, roMultiCheck, roMultiGo };
void moveItem (cell *from, cell *to, bool activateYendor);
void uncoverMines(cell *c, int lev, int dist);
void killMonster(cell *c, eMonster who_killed, flagtype flags = 0);
void toggleGates(cell *ct, eWall type, int rad);
bool destroyHalfvine(cell *c, eWall newwall = waNone, int tval = 6);
void buildCrossroads2(cell *c);
bool isHaunted(eLand l);
heptagon *createAlternateMap(cell *c, int rad, hstate firststate, int special=0);
void generateAlts(heptagon *h);
void setdist(cell *c, int d, cell *from);
void checkOnYendorPath();
void killThePlayerAt(eMonster m, cell *c, flagtype flags);
bool notDippingFor(eItem i);
bool collectItem(cell *c2, bool telekinesis = false);
void castLightningBolt(struct cellwalker lig);
bool movepcto(int d, int subdir = 1, bool checkonly = false);
void stabbingAttack(cell *mf, cell *mt, eMonster who, int bonuskill = 0);
bool earthMove(cell *from, int dir);
void messageKill(eMonster killer, eMonster victim);
void moveMonster(cell *ct, cell *cf);
int palaceHP();
void placeLocalOrbs(cell *c);
int elementalKills();
bool elementalUnlocked();
bool trollUnlocked();
bool isMultitile(eMonster m);
void checkFreedom(cell *cf);
int rosedist(cell *c);
bool canPushStatueOn(cell *c);

namespace hive { void createBugArmy(cell *c); }
namespace whirlpool { void generate(cell *wto); }
namespace whirlwind { void generate(cell *wto); }
namespace mirror { 
  static const int SPINSINGLE = 1;
  static const int SPINMULTI = 2;
  static const int GO = 4;
  static const int ATTACK = 8;
    
  void act(int dir, int flags);
  }

int neighborId(cell *c1, cell *c2);

struct movedir { 
  int d; // 0 to 6, or one of the following -- warning: not used consistently
  #define MD_WAIT (-1)
  #define MD_DROP (-2)
  #define MD_UNDECIDED (-3)
  #define MD_USE_ORB (-4)
  int subdir; // for normal movement (0-6): turn left or right
  cell *tgt;  // for MD_USE_ORB: target cell
  };

inline bool movepcto(const movedir& md) { return movepcto(md.d, md.subdir); }

void activateActiv(cell *c, bool msg);

// shmup

struct charstyle {
  int charid;
  unsigned skincolor, haircolor, dresscolor, swordcolor, dresscolor2, uicolor;
  };

string csname(charstyle& cs);
void initcs(charstyle& cs);
void savecs(FILE *f, charstyle& cs, int vernum);
void loadcs(FILE *f, charstyle& cs, int vernum);

#define MAXPLAYER 7
#define MAXJOY 8

#define MAXBUTTON 64
#define MAXAXE 16
#define MAXHAT 4

namespace multi {

  extern bool shmupcfg;
  extern bool alwaysuse;
  void recall();
  extern cell *origpos[MAXPLAYER], *origtarget[MAXPLAYER];
  extern int players;
  extern cellwalker player[MAXPLAYER];
  extern bool flipped[MAXPLAYER];
  cell *mplayerpos(int i);
  
  extern vector<int> revive_queue; // queue for revival

  extern movedir whereto[MAXPLAYER]; // player's target cell

  extern int cpid; // player id -- an extra parameter for player-related functions
  extern int cpid_edit; // cpid currently being edited

  // treasure collection, kill, and death statistics
  extern int treasures[MAXPLAYER], kills[MAXPLAYER], deaths[MAXPLAYER];

  struct config {
    int players;
    int subconfig;
    int setwhat;
    char keyaction[512];
    char joyaction[MAXJOY][MAXBUTTON];
    char axeaction[MAXJOY][MAXAXE];
    char hataction[MAXJOY][MAXHAT][4];
    int  deadzoneval[MAXJOY][MAXAXE];
    };

  void saveConfig(FILE *f);
  void loadConfig(FILE *f);
  void initConfig();
   
  charstyle scs[MAXPLAYER];

  bool playerActive(int p);
  int activePlayers();
  cell *multiPlayerTarget(int i);
  void checklastmove();
  void leaveGame(int i);
  void showShmupConfig();
  }

namespace shmup {
  using namespace multi;
  void recall();
  extern bool on;
  extern bool safety;
  extern int curtime;
  void clearMonsters();
  void clearMemory();
  void init();
  void teleported();
  extern struct monster* mousetarget;
  extern eItem targetRangedOrb(orbAction a);
  void degradeDemons();
  void killThePlayer(eMonster m);
  void killThePlayer(eMonster m, int i);
  void visibleFor(int t);
  bool verifyTeleport();
  bool dragonbreath(cell *dragon);

  void shmupDrownPlayers(cell *c);

  cell *playerpos(int i);

  bool playerInBoat(int i);
  void destroyBoats(cell *c);
  bool boatAt(cell *c);
  
  void virtualRebase(cell*& base, transmatrix& at, bool tohex);
  void virtualRebase(shmup::monster *m, bool tohex);
  void fixStorage();
  void addShmupHelp(string& out);
  }

// graph

extern int msgscroll;

void showMissionScreen();

void restartGraph();
void resetmusic();

void drawFlash(cell* c);
void drawBigFlash(cell* c);
void drawParticle(cell *c, int col, int maxspeed = 100);
void drawParticles(cell *c, int col, int qty, int maxspeed = 100);
void drawFireParticles(cell *c, int qty, int maxspeed = 100);
int firecolor(int phase);

void drawLightning();
void drawSafety();
void restartGraph();
void movepckeydir(int);

void centerpc(ld aspd);

void displayButton(int x, int y, const string& name, int key, int align, int rad = 0);
void displayColorButton(int x, int y, const string& name, int key, int align, int rad, int color, int color2 = 0);
inline string ONOFF(bool b) { return XLAT(b ? "ON" : "OFF"); }
int darkened(int c);
extern int getcstat;
bool displaychr(int x, int y, int shift, int size, char chr, int col);
bool displayfr(int x, int y, int b, int size, const string &s, int color, int align);
bool displayfrSP(int x, int y, int sh, int b, int size, const string &s, int color, int align, int p);

bool outofmap(hyperpoint h);
void applymodel(hyperpoint H, hyperpoint& Hscr);
void drawCircle(int x, int y, int size, int color);
void fixcolor(int& col);
int displaydir(cell *c, int d);
hyperpoint gethyper(ld x, ld y);
void resetview(); extern heptspin viewctr; extern cell *centerover;
void drawthemap();
void drawfullmap();
bool displaystr(int x, int y, int shift, int size, const char *str, int color, int align);
bool displaystr(int x, int y, int shift, int size, const string& str, int color, int align);

extern int darken, inmirrorcount;
void calcparam();

#if CAP_SDL
int& qpixel(SDL_Surface *surf, int x, int y);
void setvideomode();
void saveHighQualityShot(const char *fname = NULL, const char *caption = NULL, int fade = 255);
#endif

#if CAP_CONFIG
void saveConfig();
#endif

extern hyperpoint mouseh;

extern hyperpoint ccenter;
extern ld crad;

extern bool mousepressed, anyshiftclick;
extern string help;

extern function<void()> help_delegate;

#define HELPFUN(x) (help_delegate = x, "HELPFUN")

struct videopar {
  ld scale, eye, alpha, sspeed, mspeed, yshift, camera_angle;
  ld ballangle, ballproj;
  int mobilecompasssize;
  int aurastr, aurasmoothen;

  bool full;
  bool goteyes;  // for rendering
  bool goteyes2; // for choosing colors
  int graphglyph; // graphical glyphs
  bool darkhepta;
  int shifttarget;
  
  int xres, yres, framelimit;
  
  int xscr, yscr;
  
  // paramaters calculated from the above
  int xcenter, ycenter;
  int radius;
  ld alphax, beta;
  
  bool grid;
  int particles;
  
  int fsize;
  int flashtime;
  
  int wallmode, monmode, axes;
  bool revcontrol;

  // for OpenGL
  float scrdist;
  
  bool usingGL;
  int antialias;
  #define AA_NOGL      1
  #define AA_VERSION   2
  #define AA_LINES     4
  #define AA_POLY      8
  #define AA_LINEWIDTH 16
  #define AA_FONT      32
  #define AA_MULTI     64
  #define AA_MULTI16   128 // not configurable
  ld linewidth;

  int joyvalue, joyvalue2, joypanthreshold;
  ld joypanspeed;
  
  charstyle cs;
  
  bool samegender; // same gender for the Princess?
  int language;
  
  int killreduction, itemreduction, portreduction;
  
  multi::config scfg;
  
  int steamscore;
  bool drawmousecircle; // draw the circle around the mouse
  bool skipstart;       // skip the start menu
  int quickmouse;       // quick mouse on the map
  };

extern videopar vid;

extern vector< function<void()> > screens;

template<class T> void pushScreen(T& x) { screens.push_back(x); } 
inline void popScreen() { screens.pop_back(); }
inline void popScreenAll() { while(size(screens)>1) popScreen(); }

extern transmatrix View; // current rotation, relative to viewctr
extern transmatrix cwtV; // player-relative view

extern cell *mouseover, *mouseover2;
extern string mouseovers;

extern struct SDL_Surface *s;

namespace mapeditor { 
  extern bool drawplayer; 
  extern char whichPattern, whichShape;
  extern char whichCanvas;
  extern int displaycodes;
  int generateCanvas(cell *c);
  void applyModelcell(cell *c);
  int realpattern(cell *c);
  int patterndir(cell *c, char w = whichPattern);
  int subpattern(cell *c);
  extern cell *drawcell;
  void initdraw(cell *c); 
  void showMapEditor();
  void showDrawEditor();
  }

namespace rug {
  extern bool rugged;
  extern bool renderonce;
  extern bool rendernogl;
  extern int  texturesize;
  extern ld   scale;
#if CAP_RUG
  void show();
  void init();
  void close();
  void actDraw();
  void select();
  void buildVertexInfo(cell *c, transmatrix V);
#endif
  }

#define HASLINEVIEW

namespace conformal {
  extern bool on;
  extern vector<pair<cell*, eMonster> > killhistory;
  extern vector<pair<cell*, eItem> > findhistory;  
  extern vector<cell*> movehistory;
  extern bool includeHistory;
  extern int rotation;
  extern bool autoband;
  extern bool autobandhistory;
  extern bool dospiral;
  extern ld lvspeed;
  extern int bandsegment;
  extern int bandhalf;  
  
  void create();
  void clear();
  void show();
  void apply();
  void movetophase();
  void renderAutoband();

  extern vector<shmup::monster*> v;
  extern double phase;
  void applyIB();
  }
  
namespace polygonal {
  static const int MSI = 120;
  extern int SI;
  extern ld STAR;
  extern int deg;
  extern ld coefr[MSI], coefi[MSI];
  extern int maxcoef, coefid;
  void solve();
  pair<ld, ld> compute(ld x, ld y);
  }

void selectEyeGL(int ed);
void selectEyeMask(int ed);
extern int ticks;
void setGLProjection();

// passable flags

#define P_MONSTER    (1<<0)  // can move through monsters
#define P_MIRROR     (1<<1)  // can move through mirrors
#define P_REVDIR     (1<<2)  // reverse direction movement
#define P_WIND       (1<<3)  // can move against the wind
#define P_GRAVITY    (1<<4)  // can move against the gravity
#define P_ISPLAYER   (1<<5)  // player-only moves (like the Round Table jump)
#define P_ONPLAYER   (1<<6)  // always can step on the player
#define P_FLYING     (1<<7)  // is flying
#define P_BULLET     (1<<8)  // bullet can fly through more things
#define P_MIRRORWALL (1<<9)  // mirror images go through mirror walls
#define P_JUMP1      (1<<10) // first part of a jump
#define P_JUMP2      (1<<11) // second part of a jump
#define P_TELE       (1<<12) // teleport onto
#define P_BLOW       (1<<13) // Orb of Air -- blow, or push
#define P_AETHER     (1<<14) // aethereal
#define P_FISH       (1<<15) // swimming
#define P_WINTER     (1<<16) // fire resistant
#define P_USEBOAT    (1<<17) // can use boat
#define P_NOAETHER   (1<<18) // disable AETHER
#define P_FRIENDSWAP (1<<19) // can move on friends (to swap with tem)
#define P_ISFRIEND   (1<<20) // is a friend (can use Empathy + Winter/Aether/Fish combo)
#define P_LEADER     (1<<21) // can push statues and use boats
#define P_MARKWATER  (1<<22) // mark Orb of Water as used
#define P_EARTHELEM  (1<<23) // Earth Elemental
#define P_WATERELEM  (1<<24) // Water Elemental
#define P_IGNORE37   (1<<25) // ignore the triheptagonal board
#define P_CHAIN      (1<<26) // for chaining moves with boats
#define P_DEADLY     (1<<27) // suicide moves allowed
#define P_ROSE       (1<<28) // rose smell
#define P_CLIMBUP    (1<<29) // allow climbing up
#define P_CLIMBDOWN  (1<<30) // allow climbing down
#define P_REPTILE    (1<<31) // is reptile

bool passable(cell *w, cell *from, flagtype flags);

bool isElemental(eLand l);
int coastval(cell *c, eLand base);
int getHauntedDepth(cell *c);
eLand randomElementalLand();
extern eLand euland[65536];
bool notDippingForExtra(eItem i, eItem x);
void placePrizeOrb(cell *c);
int hivehard();
eMonster randomHyperbug();
void wandering();
bool isSealand(eLand l);
int newRoundTableRadius();
bool grailWasFound(cell *c);
extern bool buggyGeneration;
int buildIvy(cell *c, int children, int minleaf);
int celldistAltRelative(cell *c);
int roundTableRadius(cell *c);
cell *chosenDown(cell *c, int which, int bonus, cellfunction* cf = &coastvalEdge);
eLand pickLandRPM(eLand old);
bool bearsCamelot(eLand l);

extern bool safety;

#define SAGEMELT .1
#define TEMPLE_EACH 6
#define PT(x, y) ((tactic::on || quotient == 2) ? (y) : inv::on ? min(2*(y),x) : (x))
#define ROCKSNAKELENGTH 50
#define WORMLENGTH 15
#define PUREHARDCORE_LEVEL 10
#define PRIZEMUL 7

#define INF  9999
#define INFD 20
#define PINFD 125
#define BARLEV ((ISANDROID||ISIOS||ISFAKEMOBILE||purehepta)?9:10)
#define BUGLEV 15
// #define BARLEV 9

bool isKillable(cell *c);
bool isKillableSomehow(cell *c);

bool isAlchAny(eWall w);
bool isAlchAny(cell *c);

#define YDIST 101
#define MODECODES 255
  
extern cellwalker cwt; // player character position
extern int sval;

extern int items[ittypes], kills[motypes], explore[10], exploreland[10][landtypes], landcount[landtypes];

typedef int modecode_t;
extern map<modecode_t, array<int, ittypes> > hiitems;

extern eLand firstland, specialland;
bool pseudohept(cell *c);
bool pureHardcore();
extern int cheater;
int airdist(cell *c);
bool eq(short a, short b);
extern vector<cell*> dcal;   // queue for cpdist
bool isPlayerOn(cell *c);
bool isFriendly(eMonster m);
bool isFriendly(cell *c);
bool isChild(cell *w, cell *killed); // is w killed if killed is killed?

static const int NO_TREASURE = 1;
static const int NO_YENDOR = 2;
static const int NO_GRAIL = 4;
static const int NO_LOVE = 8;

int gold(int no = 0);
int tkills();
bool hellUnlocked();

bool markOrb(eItem it); // mark the orb as 'used', return true if exists
bool markEmpathy(eItem it); // mark both the given orb and Empathy as 'used', return true if exists
bool markEmpathy2(eItem it); // as above, but next turn

bool isMimic(eMonster m);
bool isMimic(cell *c);

void fallMonster(cell *c, flagtype flags = 0); // kill monster due to terrain

bool attackMonster(cell *c, flagtype flags, eMonster killer);

bool isWorm(eMonster m);
bool isWorm(cell *c);
void empathyMove(cell *c, cell *cto, int dir);
bool isIvy(eMonster m);
bool isIvy(cell *c);

#define GUNRANGE 3

// 0 = basic treasure, 1 = other item, 2 = power orb, 3 = not an item
#define IC_TREASURE 0
#define IC_OTHER 1
#define IC_ORB 2
#define IC_NAI 3

bool playerInPower();
void activateFlash();
void activateLightning();
bool markOrb(eItem it);
bool markOrb2(eItem it);
void drainOrb(eItem it, int target = 0);
void useupOrb(eItem it, int qty);

void initgame();
bool haveRangedTarget();
eItem targetRangedOrb(cell *c, orbAction a);
void reduceOrbPowers();
int realstuntime(cell *c);

extern bool invismove, invisfish;
bool attackingForbidden(cell *c, cell *c2);
void killOrStunMonster(cell *c2, eMonster who_killed);

void useup(cell *c); // useup thumpers/bonfires
cell *playerpos(int i);

bool makeflame(cell *c, int timeout, bool checkonly);
void bfs();
bool isPlayerInBoatOn(cell *c);
bool isPlayerInBoatOn(cell *c, int i);
void destroyBoats(cell *c, cell *cf, bool strandedToo);
extern bool showoff;
extern int lastexplore;
extern int truelotus;
extern eLand lastland;
extern time_t timerstart;
extern bool timerstopped;
bool againstRose(cell *cfrom, cell *cto);
bool withRose(cell *cfrom, cell *cto);

// loops

#define fakecellloop(ct) for(cell *ct = (cell*)1; ct; ct=NULL)

#define forCellIdAll(ct, i, cf) fakecellloop(ct) for(int i=0; i<(cf)->type && (ct=(cf)->mov[i],true); i++) 
#define forCellIdCM(ct, i, cf)  fakecellloop(ct) for(int i=0; i<(cf)->type && (ct=createMov((cf),i),true); i++) 
#define forCellIdEx(ct, i, cf)  forCellIdAll(ct,i,cf) if(ct)

#define forCellEx(ct, cf) forCellIdEx(ct,forCellEx ## __LINE__,cf)
#define forCellCM(ct, cf) forCellIdCM(ct,forCellCM ## __LINE__,cf)
#define forCellAll(ct, cf) forCellIdCM(ct,forCellAll ## __LINE__,cf)

// canAttack/moveval flags

#define AF_TOUGH             (1<<0)    // tough attacks: Hyperbugs
#define AF_MAGIC             (1<<1)    // magical attacks: Flash
#define AF_STAB              (1<<2)    // stabbing attacks (usually ignored except Hedgehogs)
#define AF_LANCE             (1<<3)    // lance attacks (used by Lancers)
#define AF_ONLY_ENEMY        (1<<4)    // only say YES if it is an enemy
#define AF_ONLY_FRIEND       (1<<5)    // only say YES if it is a friend
#define AF_ONLY_FBUG         (1<<6)    // only say YES if it is a bug_or friend
#define AF_BACK              (1<<7)    // backward attacks (ignored except Viziers and Flailers)
#define AF_APPROACH          (1<<8)    // approach attacks (ignored except Lancers)
#define AF_IGNORE_UNARMED    (1<<9)    // ignore the UNARMED flag
#define AF_NOSHIELD          (1<<10)   // ignore the shielded status
#define AF_GETPLAYER         (1<<11)   // check for player (replace m2 with moPlayer for player position)
#define AF_GUN               (1<<12)   // revolver attack
#define AF_FAST              (1<<13)   // fast attack
#define AF_EAT               (1<<17)   // eating attacks from Worm-likes

#define MF_NOATTACKS         (1<<14)   // don't do any attacks
#define MF_PATHDIST          (1<<15)   // consider pathdist for moveval
#define MF_ONLYEAGLE         (1<<16)   // do this only for Eagles
#define MF_MOUNT             (1<<18)   // don't do 
#define MF_NOFRIEND          (1<<19)   // don't do it for friends

#define AF_SWORD             (1<<20)   // big sword
#define AF_SWORD_INTO        (1<<21)   // moving into big sword
#define AF_MSG               (1<<22)   // produce a message
#define AF_ORSTUN            (1<<23)   // attackMonster: allow stunning
#define AF_NEXTTURN          (1<<24)   // next turn -- don't count shield at power 1
#define AF_FALL              (1<<25)   // death by falling
#define MF_STUNNED           (1<<26)   // edgeunstable: ignore ladders (as stunned monsters do)
#define MF_IVY               (1<<27)   // edgeunstable: ignore ivy (ivy cannot climb ivy)
#define AF_HORNS             (1<<28)   // spear attack (always has APPROACH too)
#define AF_BULL              (1<<29)   // bull attack

bool canAttack(cell *c1, eMonster m1, cell *c2, eMonster m2, flagtype flags);

extern bool chaosmode;
extern bool chaosUnlocked;
extern bool chaosAchieved;
bool isTechnicalLand(eLand l);
int getGhostcount();

void raiseBuggyGeneration(cell *c, const char *s);
void verifyMutantAround(cell *c);

#if CAP_SDL

#if CAP_PNG
#include "savepng.h"
#define IMAGEEXT ".png"
void IMAGESAVE(SDL_Surface *s, const char *fname);
#else
#define IMAGEEXT ".bmp"
#define IMAGESAVE SDL_SaveBMP
#endif

#endif

void drawscreen();

void buildAirmap();

// currently works for worms only
bool sameMonster(cell *c1, cell *c2);
cell *wormhead(cell *c);
eMonster getMount(int player_id);
eMonster haveMount();

bool isDragon(eMonster m);

// for some reason I need this to compile under OSX

#if ISMAC
extern "C" { void *_Unwind_Resume = 0; }
#endif

extern bool autocheat;
extern bool inHighQual;

void mountmove(cell *c, int spin, bool fp);
void mountmove(cell *c, int spin, bool fp, cell *ppos);
void mountswap(cell *c1, int spin1, bool fp1, cell *c2, int spin2, bool fp2);

template<class T> struct dynamicval {
  T& where;
  T backup;
  dynamicval(T& wh, T val) : where(wh) { backup = wh; wh = val; }
  dynamicval(T& wh) : where(wh) { backup = wh; }
  ~dynamicval() { where = backup; }
  };

struct stalemate1 {
  eMonster who;
  cell *moveto;
  cell *killed;
  cell *pushto;
  cell *comefrom;
  cell *swordlast[2], *swordtransit[2], *swordnext[2];
  bool isKilled(cell *c);
  stalemate1(eMonster w, cell *mt, cell *ki, cell *pt, cell *cf) : who(w), moveto(mt), killed(ki), pushto(pt), comefrom(cf) {}
  };

namespace stalemate {
  vector<stalemate1> moves;
  bool  nextturn;
  
  bool isKilled(cell *c);

  bool anyKilled();
  bool isMoveto(cell *c);
  bool isKilledDirectlyAt(cell *c);
  bool isPushto(cell *c);
  };

extern int turncount;

bool reduceOrbPower(eItem it, int cap);
bool checkOrb(eMonster m1, eItem orb);

movedir vectodir(const hyperpoint& P);

namespace sword {

  extern int angle[MAXPLAYER];

  cell *pos(cell *c, int s);
  cell *pos(int id);
  bool at(cell *where, bool noplayer = false);
  int shift(cell *c1, cell *c2);
  }

void killThePlayer(eMonster m, int id, flagtype flags);
bool attackJustStuns(cell *c2);

bool isTargetOrAdjacent(cell *c);
bool warningprotection();
bool mineMarked(cell *c);
bool minesafe();
bool hasSafeOrb(cell *c);
void placeWater(cell *c, cell *c2);
bool againstCurrent(cell *w, cell *from);

#define DEFAULTCONTROL (multi::players == 1 && !shmup::on && !multi::alwaysuse)
#define DEFAULTNOR(sym) (DEFAULTCONTROL || multi::notremapped(sym))

extern bool timerghost;

#define CAP_MENUSCALING (ISPANDORA || ISMOBILE)

#if CAP_MENUSCALING
#define displayfrZ dialog::displayzoom
#else
#define displayfrZ displayfr
#endif

namespace dialog {

  enum tDialogItem {diTitle, diItem, diBreak, diHelp, diInfo, diSlider, diBigItem};

  struct item {
    tDialogItem type;
    string body;
    string value;
    string keycaption;
    int key;
    int color, colorv, colork, colors, colorc;
    int scale;
    double param;
    int position;
    };

  extern vector<item> items;

  item& lastItem();
  extern unsigned int *palette;
  
  void addSelItem(string body, string value, int key);
  void addBoolItem(string body, bool value, int key);
  void addColorItem(string body, int value, int key);
  void openColorDialog(unsigned int& col, unsigned int *pal = palette);
  void addHelp(string body);
  void addInfo(string body, int color = 0xC0C0C0);
  void addItem(string body, int key);
  int addBreak(int val);  
  void addTitle(string body, int color, int scale);
  
  void init();
  void init(string title, int color = 0xE8E8E8, int scale = 150, int brk = 60);
  void display();

  void editNumber(ld& x, ld vmin, ld vmax, ld step, ld dft, string title, string help);
  void editNumber(int& x, int vmin, int vmax, int step, int dft, string title, string help);
  void scaleLog();
  void scaleSinh();
  void handleNavigation(int &sym, int &uni);
  bool displayzoom(int x, int y, int b, int size, const string &s, int color, int align);
  bool editingDetail();

  int handlePage(int& nl, int& nlm, int perpage);
  void displayPageButtons(int i, bool pages);
  bool handlePageButtons(int uni);
  extern bool sidedialog;
  }

void checkStunKill(cell *dest);

void clearMessages();

void resetGeometry();

namespace svg {
  void circle(int x, int y, int size, int col);
  void polygon(int *polyx, int *polyy, int polyi, int col, int outline);
  void text(int x, int y, int size, const string& str, bool frame, int col, int align);
  extern bool in;
  extern string *info;
  void render(const char *fname = NULL);
  }

extern int sightrange;

namespace halloween {
  void getTreat(cell *where);
  }

// just in case if I change my mind about when Orbs lose their power
#define ORBBASE 0

transmatrix mscale(const transmatrix& t, double fac);
transmatrix mzscale(const transmatrix& t, double fac);
extern bool ivoryz;
#define mmscale(V, x) (mmspatial ? (ivoryz ? mzscale(V,x) : mscale(V, x)) : (V))

#define SHADOW_WALL 0x60
#define SHADOW_SL   0x18
#define SHADOW_MON  0x30

bool drawMonsterType(eMonster m, cell *where, const transmatrix& V, int col, double footphase);
void drawPlayerEffects(const transmatrix& V, cell *c, bool onPlayer);

// monster movement animations

struct animation {
  int ltick;
  double footphase;
  transmatrix wherenow;
  };

// we need separate animation layers for Orb of Domination and Tentacle+Ghost,
// and also to mark Boats
#define ANIMLAYERS 3
#define LAYER_BIG   0 // for worms and krakens
#define LAYER_SMALL 1 // for others
#define LAYER_BOAT  2 // mark that a boat has moved

extern map<cell*, animation> animations[ANIMLAYERS];
extern unordered_map<cell*, transmatrix> gmatrix, gmatrix0;

void animateMovement(cell *src, cell *tgt, int layer);

// for animations which might use the same locations,
// such as replacements or multi-tile monsters
void indAnimateMovement(cell *src, cell *tgt, int layer);
void commitAnimations(int layer);

void animateReplacement(cell *a, cell *b, int layer);
void fallingFloorAnimation(cell *c, eWall w = waNone, eMonster m = moNone);
void fallingMonsterAnimation(cell *c, eMonster m, int id = multi::cpid);

// ranks:
enum PPR {
  PPR_ZERO, PPR_OUTCIRCLE, PPR_MOVESTAR,
  PPR_MINUSINF,
  PPR_BELOWBOTTOMm,
  PPR_BELOWBOTTOM,
  PPR_BELOWBOTTOMp,
  PPR_LAKEBOTTOM, PPR_HELLSPIKE,
  PPR_INLAKEWALLm, PPR_INLAKEWALL, PPR_INLAKEWALLp,
  PPR_SUBLAKELEV, PPR_LAKELEV, PPR_BOATLEV, PPR_BOATLEV2, PPR_BOATLEV3,
  PPR_LAKEWALLm, PPR_LAKEWALL, PPR_LAKEWALLp,
  PPR_FLOOR_TOWER,
  PPR_FLOOR,
  PPR_FLOOR_DRAGON,
  PPR_FLOORa, PPR_FLOORb, PPR_FLOORc, PPR_FLOORd,
  PPR_LIZEYE,
  PPR_BFLOOR,
  PPR_GFLOORa, PPR_GFLOORb, PPR_GFLOORc,
  PPR_WALLSHADOW,
  PPR_STRUCT0, PPR_STRUCT1, PPR_STRUCT2, PPR_STRUCT3,
  PPR_THORNS, PPR_WALL,
  PPR_REDWALLm, PPR_REDWALLs, PPR_REDWALLp, PPR_REDWALL,
  PPR_REDWALLm2, PPR_REDWALLs2, PPR_REDWALLp2, PPR_REDWALLt2,
  PPR_REDWALLm3, PPR_REDWALLs3, PPR_REDWALLp3, PPR_REDWALLt3,
  PPR_HEPTAMARK,
  PPR_ITEM, PPR_ITEMa, PPR_ITEMb,
  PPR_BIGSTATUE,
  PPR_GLASSm, PPR_GLASSs, PPR_GLASSp, PPR_GLASS,
  PPR_WALL3m, PPR_WALL3s, PPR_WALL3p, PPR_WALL3, PPR_WALL3A,
  PPR_HIDDEN, PPR_GIANTSHADOW,
  PPR_TENTACLE0, PPR_TENTACLE1,
  PPR_ONTENTACLE, PPR_ONTENTACLE_EYES, PPR_ONTENTACLE_EYES2,
  PPR_MONSTER_SHADOW,
  PPR_MONSTER_FOOT, PPR_MONSTER_LEG, PPR_MONSTER_GROIN,
  PPR_MONSTER_SUBWPN, PPR_MONSTER_WPN,
  PPR_MONSTER_BODY, PPR_MONSTER_ARMOR0, PPR_MONSTER_ARMOR1,
  PPR_MONSTER_CLOAK, PPR_MONSTER_NECK,
  PPR_MONSTER_HEAD, PPR_MONSTER_FACE, PPR_MONSTER_EYE0, PPR_MONSTER_EYE1,
  PPR_MONSTER_HAIR, PPR_MONSTER_HAT0, PPR_MONSTER_HAT1,
  PPR_MONSTER_HOODCLOAK1, PPR_MONSTER_HOODCLOAK2,
  PPR_STUNSTARS,
  PPR_CARRIED, PPR_CARRIEDa, PPR_CARRIEDb,
  PPR_PARTICLE, PPR_SWORDMARK, PPR_MAGICSWORD, PPR_MISSILE, 
  PPR_MINEMARK, PPR_ARROW,
  PPR_LINE, PPR_TEXT, PPR_CIRCLE,
  PPR_MAX
  };
  
void ShadowV(const transmatrix& V, const struct hpcshape& bp, int prio = PPR_MONSTER_SHADOW);

#define OUTLINE_NONE     0x000000FF
#define OUTLINE_FRIEND   0x00FF00FF
#define OUTLINE_ENEMY    0xFF0000FF
#define OUTLINE_TREASURE 0xFFFF00FF
#define OUTLINE_ORB      0xFF8000FF
#define OUTLINE_OTHER    0xFFFFFFFF
#define OUTLINE_DEAD     0x800000FF
#define OUTLINE_TRANS    0
#define OUTLINE_DEFAULT  ((bordcolor << 8) + 0xFF)
#define OUTLINE_FORE     ((forecolor << 8) + 0xFF)
#define OUTLINE_BACK     ((backcolor << 8) + 0xFF)

extern bool audio;
extern string musiclicense;
extern string musfname[landtypes];
extern int musicvolume, effvolume;
void initAudio();
bool loadMusicInfo();
void handlemusic();
void playSeenSound(cell *c);
void playSound(cell *c, const string& fname, int vol = 100);

inline string pick123() { return cts('1' + rand() % 3); }
inline string pick12() { return cts('1' + rand() % 2); }

bool playerInBoat(int i);

extern int lowfar;
extern bool wmspatial, wmescher, wmplain, wmblack, wmascii;
extern bool mmspatial, mmhigh, mmmon, mmitem;
extern int maxreclevel, reclevel;

string explain3D(ld *param);

extern int detaillevel;
extern bool quitmainloop;

enum eGlyphsortorder {
  gsoFirstTop, gsoFirstBottom,
  gsoLastTop, gsoLastBottom,
  gsoLand, gsoValue,
  gsoMAX
  };

extern eGlyphsortorder glyphsortorder;

#if CAP_ROGUEVIZ
namespace rogueviz { 
  extern bool on;
  string describe(shmup::monster *m);
  void describe(cell *c);
  void activate(shmup::monster *m);
  void drawVertex(const transmatrix &V, cell *c, shmup::monster *m);
  bool virt(shmup::monster *m);
  void turn(int delta);
  void drawExtra();
  void fixparam();
  int readArgs();
  void close();
  void mark(cell *c);
  void showMenu();
  string makehelp();
  }
#endif

void explodeMine(cell *c);
bool mayExplodeMine(cell *c, eMonster who);

int gravityLevel(cell *c);
void fullcenter();
void movecost(cell* from, cell *to);
void checkmove();

transmatrix eumove(int x, int y);
transmatrix eumovedir(int d);

int reptilemax();

extern bool mousing;
#define IFM(x) (mousing?"":x)

extern cell *recallCell;

extern eLand cheatdest;
void cheatMoveTo(eLand l);

extern unsigned backcolor, bordcolor, forecolor;

extern bool overgenerate;
void doOvergenerate();

void collectMessage(cell *c2, eItem which);

namespace quotientspace {
  void build();
  void clear();
  extern vector<int> connections;
  }

void killFriendlyIvy();

void pushdown(cell *c, int& q, const transmatrix &V, double down, bool rezoom, bool repriority);

extern bool viewdists;

void preventbarriers(cell *c);

bool passable_for(eMonster m, cell *w, cell *from, flagtype extra);

void beastcrash(cell *c, cell *beast);

int angledist(int t, int d1, int d2);
int angledist(cell *c, int d1, int d2);

void setcameraangle(bool b);

enum eModel {
  mdDisk, mdHalfplane, mdBand, mdPolygonal, mdPolynomial,
  mdEquidistant, mdEquiarea, mdBall, mdHyperboloid, mdGUARD, mdUnchanged };

#define MODELCOUNT ((int) mdGUARD)

void drawShape(pair<ld,ld>* coords, int qty, int color);

extern eModel pmodel;

int darkena(int c, int lev, int a);

#define SHSIZE 16

extern cell *shpos[MAXPLAYER][SHSIZE];
extern int cshpos;


namespace arg {
#if CAP_COMMANDLINE
  extern int argc; extern char **argv;
  
  inline void lshift() {
    argc--; argv++;
    }

  inline void shift() {
    lshift(); if(!argc) { printf("Missing parameter\n"); exit(1); }
    }

  inline char* args() { return *argv; }
  inline int argi() { return atoi(*argv); }
  inline ld argf() { return atof(*argv); }
  inline bool argis(const char *s) { return strcmp(*argv, s) == 0; }
  
  inline void init(int _argc, char **_argv) { argc=_argc-1; argv=_argv+1; }
  
  int curphase;
  
  inline void phaseerror(int x) {
    printf("Command line error: cannot read command '%s' from phase %d in phase %d\n", args(), x, curphase);
    exit(1);
    }
  
  // returned values: 0 = ok, 1 = not recognized, 2 = shift phase
  int readCommon();
  int readLocal();

// an useful macro
#define PHASE(x) { if(arg::curphase > x) phaseerror(x); else if(arg::curphase < x) return 2; }
#define PHASEFROM(x) { if(arg::curphase < x) return 2; }
  
  void read(int phase);
  
#else
  inline void read(int phase) { }
#endif
  }

extern bool generatingEquidistant;

void clearfrom(heptagon *at);
void clearHexes(heptagon *at);
void verifycells(heptagon *at);
int zebra40(cell *c);
cell *createMov(cell *c, int d);

#if CAP_TOUR
namespace tour {
  extern bool on;
  extern string tourhelp;
  extern string slidecommand;
  extern int currentslide;
  
  enum presmode { 
    pmStartAll = 0,
    pmStart = 1, pmFrame = 2, pmStop = 3, pmKey = 4, pmRestart = 5,
    pmAfterFrame = 6,
    pmGeometry = 11, pmGeometryReset = 13, pmGeometryStart = 15
    };

  void setCanvas(presmode mode, char canv);

  void presentation(presmode mode);
  void checkGoodLand(eLand l);
  int getid();
  
  extern function<eLand(eLand)> getNext;
  extern function<bool(eLand)> quickfind;
  extern function<bool(eLand)> showland;

  void start();

  struct slide { 
    const char *name; int unused_id; int flags; const char *help; 
    function<void(presmode mode)> action;
    } ;
  
  extern slide *slides;
  extern slide default_slides[];

  static const int LEGAL_NONE=0;
  static const int LEGAL_UNLIMITED=1;
  static const int LEGAL_HYPERBOLIC=2;
  static const int LEGAL_ANY=3;
  static const int LEGAL_NONEUC=4;
  static const int QUICKSKIP=8;
  static const int FINALSLIDE=16;
  static const int QUICKGEO=32;
  static const int SIDESCREEN = 64;
  
  extern slide slideHypersian;
  extern slide slideExpansion;

  namespace ss {
    void showMenu();
    void list(slide*);
    }

  };
#endif

namespace rogueviz {
  extern bool rog3;
  extern bool rvwarp;
#if CAP_TOUR
  namespace rvtour {
    extern tour::slide rvslides[];
    }
#endif
  };

extern bool doCross;
void optimizeview();

extern int pngres;
extern int pngformat;

extern bool noGUI;
extern bool dronemode;

extern ld whatever;

namespace sm {
  static const int NORMAL = 1;
  static const int MISSION = 2;
  static const int HELP = 4;
  static const int MAP = 8;
  static const int DRAW = 16;
  static const int NUMBER = 32;
  static const int SHMUPCONFIG = 64;
  static const int OVERVIEW = 128;
  static const int SIDE = 256;
  static const int DOTOUR = 512;
  static const int CENTER = 1024;
  static const int A3 = 2048; // affects poly
  static const int ZOOMABLE = 4096;
  };

namespace linepatterns {

  enum ePattern {
    patPalacelike,
    patPalace,
    patZebraTriangles,
    patZebraLines,
    patTriNet,
    patTriRings,
    patHepta,
    patRhomb,
    patTree,
    patAltTree,
    patVine,
    patPower,
    patNormal,
    patTrihepta,
    patBigTriangles,
    patBigRings
    };
  
  void clearAll();
  void setColor(ePattern id, int col);
  void drawAll();
  void showMenu();
  };

transmatrix ddspin(cell *c, int d, int bonus = 0);
bool doexiton(int sym, int uni);
void switchFullscreen();
string turnstring(int i);
int celldistance(cell *c1, cell *c2);
bool behindsphere(const transmatrix& V);
extern hyperpoint pirateCoords;

bool mouseout();

bool againstWind(cell *c2, cell *c1); // to, from

transmatrix atscreenpos(ld x, ld y, ld size);

hyperpoint mirrorif(const hyperpoint& V, bool b);

#define SETMOUSEKEY 5000
extern char mousekey;
extern char newmousekey;
void displaymm(char c, int x, int y, int rad, int size, const string& title, int align);

bool canPushThumperOn(cell *tgt, cell *thumper, cell *player);
void pushThumper(cell *th, cell *cto);

template<class T> T pick(T x, T y) { return hrand(2) ? x : y; }
template<class T> T pick(T x, T y, T z) { switch(hrand(3)) { case 0: return x; case 1: return y; case 2: return z; } return x; }
template<class T> T pick(T x, T y, T z, T v) { switch(hrand(4)) { case 0: return x; case 1: return y; case 2: return z; case 3: return v; } return x; }
template<class T, class... U> bool among(T x, T y) { return x == y; }
template<class T, class... U> bool among(T x, T y, U... u) { return x==y || among(x,u...); }

eLand getNewSealand(eLand old);
bool createOnSea(eLand old);

namespace inv {
  extern bool on;
  extern bool usedForbidden;
  extern int remaining[ittypes];
  void compute();
  void applyBox(eItem it);

  extern int incheck;  
  void check(int delta);
  void show();
  }

bool drawItemType(eItem it, cell *c, const transmatrix& V, int icol, int ticks, bool hidden);

void initquickqueue();
void quickqueue();
int darkenedby(int c, int lev);
extern int mousex, mousey;
string generateHelpForItem(eItem it);
bool graphglyph();
extern bool hiliteclick;
extern int antialiaslines;
extern int ringcolor;

#include <functional>

template<class T> class hookset : public map<int, function<T>> {};
typedef hookset<void()> *purehookset;

template<class T, class U> int addHook(hookset<T>*& m, int prio, const U& hook) {
  if(!m) m = new hookset<T> ();
  while(m->count(prio)) {
    prio++;
    }
  (*m)[prio] = hook;
  return 0;
  }

extern purehookset hooks_frame, hooks_stats, clearmemory, hooks_config;

template<class T, class... U> void callhooks(hookset<T> *h, U... args) {
  if(h) for(auto& p: *h) p.second(args...);
  }

template<class T, class V, class... U> V callhandlers(V zero, hookset<T> *h, U... args) {
  if(h) for(auto& p: *h) {
    auto z = p.second(args...);
    if(z != zero) return z;
    }
  return zero;
  }

extern hookset<bool(int sym, int uni)> *hooks_handleKey;
extern hookset<void(cell *c, const transmatrix& V)> *hooks_drawcell;
extern hookset<bool(int argc, char** argv)> *hooks_main;
extern hookset<int()> *hooks_args;
extern hookset<eLand(eLand)> *hooks_nextland;

// hooks to extend HyperRogue with an external program
// start compilation from another file which defines EXTRA_..., includes
// hyper.cpp, then defines the necessary functions

extern ld shiftmul;
void initcs(charstyle &cs);
charstyle& getcs(int id = multi::cpid);

struct msginfo {
  int stamp;
  char flashout;
  char spamtype;
  int quantity;
  string msg;
  };

extern vector<msginfo> msgs;
void flashMessages();

extern int lightat, safetyat;

int watercolor(int phase);
bool doHighlight();
string buildHelpText();
string buildCredits();
void setAppropriateOverview();
bool quitsaves();
extern bool sidescreen;

static const char* COLORBAR = "###";
int textwidth(int siz, const string &str);
#define GLERR(call) glError(call, __FILE__, __LINE__)

extern bool gtouched, mousepressed, mousemoved, actonrelease;
extern bool inslider;

#if CAP_ROGUEVIZ
#define DOSHMUP (shmup::on || rogueviz::on)
#else
#define DOSHMUP shmup::on
#endif

extern bool outoffocus;
extern int frames;
extern transmatrix playerV;
extern bool didsomething;
extern void drawStats();
extern int calcfps();
extern int distcolors[8];

extern eItem orbToTarget;
extern eMonster monsterToSummon;

void panning(hyperpoint hf, hyperpoint ht);
extern transmatrix sphereflip;

void initConfig();
void loadConfig();

extern bool auraNOGL;

#if CAP_SDLJOY
extern void initJoysticks();
extern bool autojoy;
extern int joyx, joyy, panjoyx, panjoyy;
extern movedir joydir;
extern SDL_Joystick* sticks[8];
extern int numsticks;
void closeJoysticks();
#endif

void preparesort();

#if ISMOBILE==1
#define SHMUPTITLE "shoot'em up mode"
#else
#define SHMUPTITLE "shoot'em up and multiplayer"
#endif

bool dodrawcell(cell *c);
void drawcell(cell *c, transmatrix V, int spinv, bool mirrored);
extern double downspin;

extern int frameid;
extern bool leftclick;
void clearMemory();

extern function <void(int sym, int uni)> keyhandler;
void gmodekeys(int sym, int uni);

void switchGL();
void switchFullscreen();
extern int cmode;

namespace scores { void load(); }

void gotoHelp(const string& h);
void showCustomizeChar();
void showCheatMenu();
void showDisplayMode();
void showChangeMode();
void showEuclideanMenu();
void show3D();
void gameoverscreen();
void showJoyConfig();

void gamescreen(int darken);
void showMission();
void handleKeyQuit(int sym, int uni);
void handlePanning(int sym, int uni);

#if ISMOBILE==1
namespace leader { void showMenu(); void handleKey(int sym, int uni); }
#endif

bool needConfirmation();

extern const char* geometrynames_short[gGUARD];

namespace mirror {
  cellwalker reflect(const cellwalker& cw);
  }

bool inmirror(eLand l);
bool inmirror(cell *c);
bool inmirror(const cellwalker& cw);

void queuemarkerat(const transmatrix& V, int col);

void check_total_victory();
void applyBoxNum(int& i, string name = "");
extern int hinttoshow;

bool isShmupLifeOrb(eItem it);
int orbcharges(eItem it);

int gradient(int c0, int c1, ld v0, ld v, ld v1);

struct hint {
  time_t last;
  function<bool()> usable;
  function<void()> display;
  function<void()> action;  
  };

extern hint hints[];
int counthints();

void gainShard(cell *c2, const char *msg);

int textwidth(int siz, const string &str);
#if CAP_GL
int gl_width(int size, const char *s);
#endif

#ifdef ISMOBILE
extern int andmode;
#endif

void addaura(const hyperpoint& h, int col, int fd);
void addauraspecial(const hyperpoint& h, int col, int dir);

void drawBug(const cellwalker& cw, int col);

void mainloop();
void mainloopiter();
extern bool showstartmenu;
void selectLanguageScreen();

bool inscreenrange(cell *c);
bool allowIncreasedSight();

static bool orbProtection(eItem it) { return false; } // not implemented

namespace windmap {
  void create();
  }

extern int wavephase;
