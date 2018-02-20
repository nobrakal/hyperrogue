// -- mission screen --

bool quitsaves() { return (items[itOrbSafety] && havesave); }

bool needConfirmation() {
  return canmove && (gold() >= 30 || tkills() >= 50) && !cheater && !quitsaves();
  }

int msgscroll = 0;

string timeline() {
    int timespent = (int) (savetime + (timerstopped ? 0 : (time(NULL) - timerstart)));
  char buf[20];
  sprintf(buf, "%d:%02d", timespent/60, timespent % 60);
  return 
    shmup::on ? 
      XLAT("%1 knives (%2)", its(turncount), buf)
    :
      XLAT("%1 turns (%2)", its(turncount), buf);
  }

void noaction() {}

function<void()> cancel = noaction;

string circlesizestr(int r) {
  computeSizes();
  string s;
  int last = lastsize;
  if(r <= last)
    return llts(circlesize[r]);
  else {
    double d = log(circlesize[last]) + (log(circlesize[last]) - log(circlesize[last-1]))*(r-last);
    int dlost = 0;
    while(d > 10 * log(10)) d -= log(10), dlost++;
    char buf[300]; sprintf(buf, "%.0f", exp(d)); 
    string ss = XLAT("about ") + buf;
    while(dlost % 9) dlost--, ss += '0';
    for(int r = 0; r < 50 && dlost; r++) dlost -= 9, ss += " 000000000";
    if(dlost) ss += XLAT(" (%1 more digits)", its(dlost));
    return ss;
    }
  }

hint hints[] = {

  {
    0,
    []() { 
      return !inv::on && geometry == gNormal && items[localTreasureType()] >= 18;
      },    
    []() { 
       dialog::addHelp(XLAT(
        "If you collect too many treasures in a given land, it will become "
        "extremely dangerous. Try other lands once you have enough!"));
      },
    noaction},

  {
    0,
    []() { 
      return !ISMOBILE;
      },    
    []() { 
       dialog::addHelp(XLAT(
        "Remember that you can right click almost anything for more information."));
#if ISMAC
       dialog::addHelp(XLAT(
         "(You can also use right Shift)\n\n"));
#endif
      },
    noaction},

  {
    0,
    []() { return !canmove; },
    []() { 
      dialog::addHelp(XLAT(
        "Want to understand the geometry in HyperRogue? Try the Tutorial!"
        ));
      dialog::addBreak(50);
      dialog::addItem(XLAT("Tutorial"), 'z');
      },
    []() {
#if CAP_TOUR    
      tour::start();
#else
      addMessage("Not in this version");
#endif
      }},

  {
    0,
    []() { return !inv::on; },
    []() { 
      dialog::addHelp(XLAT(
        "Collecting 25 treasures in a given land may be dangerous, "
        "but allows magical Orbs of this land to appear in other places!"
        ));
      },
    noaction},

  {
    0,
    []() { return !canmove; },
    []() { 
      dialog::addInfo(XLAT(
        "Press ESC to view this screen during the game."
        ));
      },
    noaction
    },
  {
    0,
    []() { return true; },
    []() { 
      dialog::addInfo(XLAT(
#if ISMOBILE==1
        "The 'world overview' shows all the lands in HyperRogue."
#else
        "Press 'o' to see all the lands in HyperRogue."
#endif
        ));
      dialog::addBreak(50);
      dialog::addItem(XLAT("world overview"), 'z');
      },
    []() {
      setAppropriateOverview();
      }},
  {
    0,
    []() { return !canmove; },
    []() { 
      dialog::addHelp(XLAT(
        "Want another type of game? Want more challenge?\n"
        "HyperRogue has many special modes and challenges that "
        "significantly change the gameplay. Try them!"
        ));
      dialog::addBreak(50);
      dialog::addItem(XLAT("special game modes"), 'z');
      },
    []() {
      pushScreen(showChangeMode);
      }},

  {
    0,
    []() { return true; },
    []() { 
      dialog::addInfo(XLAT(
        "Hyperbolic geometry can be shown in many ways."
        ));
      dialog::addBreak(50);
      dialog::addItem(XLAT("special display modes"), 'z');
      },
    []() {
      pushScreen(showDisplayMode);
      }},

  {
    0,
    []() { return !canmove && !inv::on; },
    []() { 
      dialog::addHelp(XLAT(
        "You do not want to lose the game from a single mistake?\n"
        "Do you want to use the Orbs strategically?\n"
        "Try the Orb Strategy mode!")
        );
      dialog::addBreak(50);
#ifdef INF
      dialog::addItem(XLAT("Orb Strategy mode"), 'z');
#endif
      },
    []() {
#if CAP_INV
      restartGame('i');
#endif
      }
    },
  {
    0,
    []() { return CAP_RUG && geometry == gNormal; },
    []() { 
      dialog::addHelp(XLAT(
        "Do you think you are playing on a ball? "
        "This is far from the truth!\n"
        ));
      dialog::addBreak(50);
      dialog::addItem(XLAT("hypersian rug mode"), 'z');
      },
    [] () {
#if CAP_RUG
      popScreen();
      int wm, mm;
      rug::init();
      wm = vid.wallmode;
      mm = vid.monmode;
      vid.wallmode = 3;
      vid.monmode = 2;
      cancel = [wm, mm] () { 
        rug::close();
        vid.wallmode = wm;
        vid.monmode = mm;
        };
#endif
      }
    },

  {
    0,
    []() { return !canmove && geometry == gNormal && celldist(cwt.c) >= 50; },
    []() { 
      dialog::addHelp(XLAT(
        "Did you know that the path you take during the game "
        "is usually very close to a straight line?\n"
        ));
      dialog::addBreak(50);
      dialog::addItem(XLAT("Show me!"), 'z');
      },
    [] () {
      popScreen();
      auto m = pmodel;
      pmodel = mdBand;
      int r = conformal::rotation;
      bool h = conformal::includeHistory;
      conformal::rotation = 0;
      conformal::includeHistory = true;
      conformal::create();
      cancel = [m,r,h] () { 
        conformal::clear(); pmodel = m; 
        conformal::rotation = r;
        conformal::includeHistory = h;
        fullcenter(); };
      }
    },

  {
    0,
    []() { return !canmove && geometry == gNormal && celldist(cwt.c) >= 50; },
    []() {
      int c = celldist(cwt.c);
      string s = circlesizestr(c);
      dialog::addHelp(XLAT(
        "You are %1 cells away from the starting point, or "
        "the place where you used an Orb of Safety last time. "
        "There are %2 such cells.\n",
        its(c), s
        ));
      dialog::addBreak(50);
      dialog::addItem(XLAT("expansion"), 'z');
      },
    [] () {
      viewdists = !viewdists;
      popScreen();
      cancel = [] () { viewdists = false; };
      }
    },

  {
    0,
    []() { 
      if(canmove) return false;
      time_t t = time(NULL);
      struct tm tm = *localtime(&t);
      int month = tm.tm_mon + 1;
      int day = tm.tm_mday;
      if(month == 10 && day >= 24) return true;
      if(month == 11 && day <= 7) return true;
      return false;
      },
    []() {
      dialog::addItem(XLAT("Halloween mini-game"), 'z');
      },
    [] () {
      if(!sphere) {
        specialland = laHalloween;
        targetgeometry = gSphere;
        restartGame('E');
        vid.alpha = 999;
        vid.scale = 998;
        }
      else {
        restartGame('E');
        vid.alpha = 1;
        vid.scale = 1;
        }
      }
    },

  {-1, []() { return false; }, noaction, noaction}
  }; 

int hinttoshow;

string contstr() {
  return XLAT(canmove ? "continue" : "see how it ended");
  }

void showMission() {

  cmode = sm::DOTOUR | sm::MISSION | sm::CENTER;
  gamescreen(1); drawStats();
  keyhandler = handleKeyQuit;

  dialog::init(
#if CAP_TOUR
    tour::on ? (canmove ? XLAT("Tutorial") : XLAT("GAME OVER")) :
#endif
    cheater ? XLAT("It is a shame to cheat!") : 
    showoff ? XLAT("Showoff mode") :
    (canmove && princess::challenge) ? XLAT("%1 Challenge", moPrincess) :
    canmove ? XLAT("Quest status") : 
    XLAT("GAME OVER"), 
    0xC00000, 200, 100
    );

  if(!peace::on)
    dialog::addInfo(XLAT("Your score: %1", its(gold())));
  if(!peace::on)
    dialog::addInfo(XLAT("Enemies killed: %1", its(tkills())));

#if CAP_TOUR
  if(tour::on) ; else 
#endif
  if(items[itOrbYendor]) {
    dialog::addInfo(XLAT("Orbs of Yendor found: %1", its(items[itOrbYendor])), iinf[itOrbYendor].color);
    dialog::addInfo(XLAT("CONGRATULATIONS!"), iinf[itOrbYendor].color);
    }
  else {
    if(tour::on)
      ;
    else if(princess::challenge) 
      dialog::addInfo(XLAT("Follow the Mouse and escape with %the1!", moPrincess));
    else if(gold() < R_30)
      dialog::addInfo(XLAT("Collect %1 $$$ to access more worlds", its(R_30)));
    else if(gold() < R_60)
      dialog::addInfo(XLAT("Collect %1 $$$ to access even more lands", its(R_60)));
    else if(!hellUnlocked())
      dialog::addInfo(XLAT("Collect at least %1 treasures in each of 9 types to access Hell", its(R_10)));
    else if(items[itHell] < R_10)
      dialog::addInfo(XLAT("Collect at least %1 Demon Daisies to find the Orbs of Yendor", its(R_10)));
    else if(size(yendor::yi) == 0)
      dialog::addInfo(XLAT("Look for the Orbs of Yendor in Hell or in the Crossroads!"));
    else 
      dialog::addInfo(XLAT("Unlock the Orb of Yendor!"));
    }
  
  if(!timerstopped && !canmove) {
    savetime += time(NULL) - timerstart;
    timerstopped = true;
    }
  if(canmove && !timerstart)
    timerstart = time(NULL);
  
  if(princess::challenge) ;
#if CAP_TOUR
  else if(tour::on) ;
#endif
  else if(peace::on) ;
  else if(tkills() < R_100)
    dialog::addInfo(XLAT("Defeat %1 enemies to access the Graveyard", its(R_100)));
  else if(kills[moVizier] == 0 && (items[itFernFlower] < U_5 || items[itGold] < U_5))
    dialog::addInfo(XLAT("Kill a Vizier in the Palace to access Emerald Mine"));
  else if(items[itEmerald] < U_5)
    dialog::addInfo(XLAT("Collect 5 Emeralds to access Camelot"));
  else if(hellUnlocked() && !chaosmode) {
    bool b = true;
    for(int i=0; i<LAND_HYP; i++)
      if(b && items[treasureType(land_hyp[i])] < R_10) {
        dialog::addInfo(
          XLAT(
            land_hyp[i] == laTortoise ? "Hyperstone Quest: collect at least %3 points in %the2" :
            "Hyperstone Quest: collect at least %3 %1 in %the2", 
            treasureType(land_hyp[i]), land_hyp[i], its(R_10)));
        b = false;
        }
    if(b) 
      dialog::addInfo(XLAT("Hyperstone Quest completed!"), iinf[itHyperstone].color);
    }
  else dialog::addInfo(XLAT("Some lands unlock at specific treasures or kills"));
  if(cheater && !autocheat) {
    dialog::addInfo(XLAT("you have cheated %1 times", its(cheater)), 0xFF2020);
    }
  else {
    dialog::addInfo(timeline(), 0xC0C0C0);
    }
  
  msgs.clear();
  if(msgscroll < 0) msgscroll = 0;
  int gls = size(gamelog) - msgscroll;
  int mnum = 0;
  for(int i=gls-5; i<gls; i++) 
    if(i>=0) {
      msginfo m;
      m.spamtype = 0;
      m.flashout = true;
      m.stamp = ticks-128*vid.flashtime-128*(gls-i);
      m.msg = gamelog[i].msg;
      m.quantity = gamelog[i].quantity;
      mnum++,
      msgs.push_back(m);
      }

  dialog::addBreak(100);

#if CAP_TOUR  
  if(!tour::on) {
    hints[hinttoshow].display();
    dialog::addBreak(100);
    }
#endif
  
  bool intour = false;
  
#if CAP_TOUR
  intour = tour::on;
#endif

  if(intour) {
#if CAP_TOUR
    if(canmove) {
      dialog::addItem(XLAT("spherical geometry"), '1');
      dialog::addItem(XLAT("Euclidean geometry"), '2');
      dialog::addItem(XLAT("more curved hyperbolic geometry"), '3');
      }
    if(!items[itOrbTeleport])
      dialog::addItem(XLAT("teleport away"), '4');
    else if(!items[itOrbAether])
      dialog::addItem(XLAT("move through walls"), '4');
    else
      dialog::addItem(XLAT("flash"), '4');
    if(canmove) {
      if(tour::slidecommand != "") 
        dialog::addItem(tour::slidecommand, '5');
      dialog::addItem(XLAT("static mode"), '6');
      dialog::addItem(XLAT("enable/disable texts"), '7');
      dialog::addItem(XLAT("next slide"), SDLK_RETURN);
      dialog::addItem(XLAT("previous slide"), SDLK_BACKSPACE);
      dialog::addItem(XLAT("list of slides"), '9');
      }
    else
      dialog::addBreak(200);
    dialog::addItem(XLAT("main menu"), 'v');
    dialog::addItem("continue", SDLK_ESCAPE);
#endif
    }
  else {
    dialog::addItem(contstr(), SDLK_ESCAPE);
    dialog::addItem(XLAT("main menu"), 'v');
    dialog::addItem(XLAT("restart"), SDLK_F5);
    if(inv::on && items[itInventory])
      dialog::addItem(XLAT("inventory"), 'i');
    #if ISMOBILE==0
    dialog::addItem(XLAT(quitsaves() ? "save" : "quit"), SDLK_F10);
    #endif
    #if CAP_ANDROIDSHARE
    dialog::addItem(XLAT("SHARE"), 's'-96);
    #endif
    dialog::addBreak(500);
    }
  
  dialog::display();

  if(mnum)
    displayfr(vid.xres/2, vid.yres-vid.fsize*(mnum+1), 2, vid.fsize/2,  XLAT("last messages:"), 0xC0C0C0, 8);  
  }

void handleKeyQuit(int sym, int uni) {
  dialog::handleNavigation(sym, uni);
  // ignore the camera movement keys

#if CAP_RUG
  if(rug::rugged && (sym == SDLK_UP || sym == SDLK_DOWN || sym == SDLK_PAGEUP || sym == SDLK_PAGEDOWN ||
    sym == SDLK_RIGHT || sym == SDLK_LEFT))
    sym = 0;
#endif

  if(sym == SDLK_RETURN || sym == SDLK_KP_ENTER || sym == SDLK_F10) quitmainloop = true;
  else if(uni == 'r' || sym == SDLK_F5) {
    restartGame();
    msgs.clear();
    }
  else if(sym == SDLK_UP || sym == SDLK_KP8 || sym == PSEUDOKEY_WHEELUP) msgscroll++;
  else if(sym == SDLK_DOWN || sym == SDLK_KP2 || sym == PSEUDOKEY_WHEELDOWN) msgscroll--;
  else if(sym == SDLK_PAGEUP || sym == SDLK_KP9) msgscroll+=5;
  else if(sym == SDLK_PAGEDOWN || sym == SDLK_KP3) msgscroll-=5;
  else if(uni == 'v') popScreenAll(), pushScreen(showMainMenu);
  else if(uni == 'z') hints[hinttoshow].action();
  else if(sym == SDLK_F3 || (sym == ' ' || sym == SDLK_HOME)) 
    fullcenter();
  else if(uni == 'o') setAppropriateOverview();
#if CAP_INV
  else if(uni == 'i' && inv::on) 
    pushScreen(inv::show);
#endif
#if CAP_SAVE
  else if(uni == 't') {
    msgs.clear();
    scores::load();
    }
  #endif
  
  else if(doexiton(sym, uni) && !didsomething) {
    popScreen();
    msgscroll = 0;
    msgs.clear();
    if(!canmove) {
      addMessage(XLAT("GAME OVER"));
      addMessage(timeline());
      }
    }
  }

int counthints() {
  for(int h=0;; h++) if(hints[h].last < 0) return h;
  }

void showMissionScreen() {
  cancel(); cancel = noaction;
  popScreenAll();
  pushScreen(showMission);
  achievement_final(false);
  msgscroll = 0;

#if CAP_TOUR
  if(!tour::on)
#endif
  {
    int ch = counthints();
    hinttoshow = ch;
    int h;
    for(h=0; h < ch; h++) {
      if(!hints[h].usable()) continue;
      if(hinttoshow == ch || hints[h].last < hints[hinttoshow].last) hinttoshow = h;
      }
    hints[hinttoshow].last = time(NULL);
    }
  
  dialog::highlight_text = contstr();
  }

