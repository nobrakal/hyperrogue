string help;

function<void()> help_delegate;

string buildHelpText() {
  DEBB(DF_GRAPH, (debugfile,"buildHelpText\n"));

#if CAP_ROGUEVIZ  
  if(rogueviz::on) return rogueviz::makehelp();
#endif
  
  string h;
  h += XLAT("Welcome to HyperRogue");
#if ISANDROID  
  h += XLAT(" for Android");
#endif
#if ISIOS
  h += XLAT(" for iOS");
#endif
  h += XLAT("! (version %1)\n\n", VER);
  
  h += XLAT(
    "You have been trapped in a strange, non-Euclidean world. Collect as much treasure as possible "
    "before being caught by monsters. The more treasure you collect, the more "
    "monsters come to hunt you, as long as you are in the same land type. The "
    "Orbs of Yendor are the ultimate treasure; get at least one of them to win the game!"
    );
  h += XLAT(" (press ESC for some hints about it).");
  h += "\n\n";
  
  if(!shmup::on && !hardcore)
    h += XLAT(
      "You can fight most monsters by moving into their location. "
      "The monster could also kill you by moving into your location, but the game "
      "automatically cancels all moves which result in that.\n\n"
      );
    
  if(inv::on)
  h += XLAT(
    inv::helptext
    );
  else
  h += XLAT(
    "There are many lands in HyperRogue. Collect 10 treasure "
    "in the given land type to complete it; this enables you to "
    "find the magical Orbs of this land, and in some cases "
    "get access to new lands. At 25 treasures "
    "this type of Orbs starts appearing in other lands as well. Press 'o' to "
    "get the details of all the Lands.\n\n");
  h += "\n\n";
    
#if ISMOBILE
  h += XLAT(
    "Usually, you move by touching somewhere on the map; you can also touch one "
    "of the four buttons on the map corners to change this (to scroll the map "
    "or get information about map objects). You can also touch the "
    "numbers displayed to get their meanings.\n"
    );
#else
  if(DEFAULTCONTROL)
    h += XLAT(
      "Move with mouse, num pad, qweadzxc, or hjklyubn. Wait by pressing 's' or '.'. Spin the world with arrows, PageUp/Down, and Home/Space. "
      "To save the game you need an Orb of Safety. Press 'v' for the main menu (configuration, special modes, etc.), ESC for the quest status.\n\n"
      );
  h += XLAT(
    "You can right click any element to get more information about it.\n\n"
    );
#if ISMAC
  h += XLAT("(You can also use right Shift)\n\n");
#endif
#endif
  h += XLAT("See more on the website: ") 
    + "http//roguetemple.com/z/hyper/\n\n";
  
#if CAP_TOUR
  h += XLAT("Try the Tutorial to help with understanding the "
    "geometry of HyperRogue (menu -> special modes).\n\n");
#endif
  
  h += XLAT("Still confused? Read the FAQ on the HyperRogue website!\n\n");
  
  return h;
  }

string buildCredits() {
  string h;
  h += XLAT("game design, programming, texts and graphics by Zeno Rogue <zeno@attnam.com>\n\n");
  if(lang() != 0)
    h += XLAT("add credits for your translation here");
#ifndef NOLICENSE
  h += XLAT(
    "released under GNU General Public License version 2 and thus "
    "comes with absolutely no warranty; see COPYING for details\n\n"
    );
#endif
  h += XLAT(
    "special thanks to the following people for their bug reports, feature requests, porting, and other help:\n\n%1\n\n",
    "Konstantin Stupnik, ortoslon, chrysn, Adam Borowski, Damyan Ivanov, Ryan Farnsley, mcobit, Darren Grey, tricosahedron, Maciej Chojecki, Marek ÄŚtrnĂˇct, "
    "wonderfullizardofoz, Piotr MigdaĹ‚, tehora, Michael Heerdegen, Sprite Guard, zelda0x181e, Vipul, snowyowl0, Patashu, phenomist, Alan Malloy, Tom Fryers, Sinquetica, _monad, CtrlAltDestroy, jruderman, "
    "Kojiguchi Kazuki, baconcow, Alan" 
    );
#ifdef EXTRALICENSE
  h += EXTRALICENSE;
#endif
#if !ISMOBILE
  h += XLAT(
    "\n\nSee sounds/credits.txt for credits for sound effects"
    );
  #endif
  if(musiclicense != "") h += musiclicense;
  return h;
  }

string pushtext(stringpar p) {
  string s = XLAT(
    "\n\nNote: when pushing %the1 off a heptagonal cell, you can control the pushing direction "
    "by clicking left or right half of the heptagon.", p);
#if !ISMOBILE
  s += XLAT(" With the keyboard, you can rotate the view for a similar effect (Page Up/Down).");
#endif
  return s;
  }

string princedesc() {
  if(princessgender() == GEN_M)
    return XLAT("Apparently a prince is kept locked somewhere, but you won't ever find him in this hyperbolic palace. ");
  else
    return XLAT("Apparently a princess is kept locked somewhere, but you won't ever find her in this hyperbolic palace. ");
  }

string helptitle(string s, int col) {
  return "@" + its(col) + "\t" + s + "\n";
  }

string princessReviveHelp() {
  if(inv::on) return "";
  string h = "\n\n" +
    XLAT("Killed %1 can be revived with Orb of the Love, after you collect 20 more $$$.", moPrincess);
  if(princess::reviveAt)
    h += "\n\n" +
    XLAT("%The1 will be revivable at %2 $$$", moPrincess, its(princess::reviveAt));
  return h;
  }

void describeOrb(string& help, const orbinfo& oi) {
  if(inv::on) return;
  eOrbLandRelation olr = getOLR(oi.orb, getPrizeLand());
  eItem tr = treasureType(oi.l);
  help += "\n\n" + XLAT(olrDescriptions[olr], cwt.c->land, tr, treasureType(cwt.c->land));
  int t = items[tr] * landMultiplier(oi.l);
  if(t >= 25)
  if(olr == olrPrize25 || olr == olrPrize3 || olr == olrGuest || olr == olrMonster || olr == olrAlways) {
    help += XLAT("\nSpawn rate (as prize Orb): %1%/%2\n", 
      its(int(.5 + 100 * orbprizefun(t))),
      its(oi.gchance));
    }
  if(t >= 10)
  if(olr == olrHub) {
    help += XLAT("\nSpawn rate (in Hubs): %1%/%2\n", 
      its(int(.5 + 100 * orbcrossfun(t))),
      its(oi.gchance));
    }
  }

string generateHelpForItem(eItem it) {

   string help = helptitle(XLATN(iinf[it].name), iinf[it].color);
   
   help += XLAT(iinf[it].help);
   
   if(it == itSavedPrincess || it == itOrbLove) if(!inv::on)
     help += princessReviveHelp();
     
   if(it == itTrollEgg)
     help += XLAT("\n\nAfter the Trolls leave, you have 750 turns to collect %the1, or it gets stolen.", it);
   
   if(it == itIvory || it == itAmethyst || it == itLotus || it == itMutant) {
     help += XLAT(
       "\n\nEasy %1 might disappear when you collect more of its kind.", it);
     if(it != itMutant) help += XLAT(
       " You need to go deep to collect lots of them.");
     }
     
#if ISMOBILE==1
   if(it == itOrbSafety)
     help += XLAT("This might be very useful for devices with limited memory.");
#else
   if(it == itOrbSafety)
     help += XLAT("Thus, it is potentially useful for extremely long games, which would eat all the memory on your system otherwise.\n");
#endif

   if(isRangedOrb(it)) {
     help += XLAT("\nThis is a ranged Orb. ");
#if ISMOBILE   
     if(vid.shifttarget&2)
       help += XLAT("\nRanged Orbs can be targeted by long touching the desired location.");
     else
       help += XLAT("\nRanged Orbs can be targeted by touching the desired location.");
#else
     if(vid.shifttarget&1)
       help += XLAT("\nRanged Orbs can be targeted by shift-clicking the desired location. ");
     else
       help += XLAT("\nRanged Orbs can be targeted by clicking the desired location. ");
     help += XLAT("You can also scroll to the desired location and then press 't'.");
#endif
     help += XLAT("\nYou can never target cells which are adjacent to the player character, or ones out of the sight range.");
     }

#if ISMOBILE
   if(it == itGreenStone)
     help += XLAT("You can touch the Dead Orb in your inventory to drop it.");
#else
   if(it == itGreenStone)
     help += XLAT("You can press 'g' or click them in the list to drop a Dead Orb.");
#endif
   if(it == itOrbLightning || it == itOrbFlash)
     help += XLAT("\n\nThis Orb is triggered on your first attack or illegal move.");
   if(it == itOrbShield)
     help += XLAT("\n\nThis Orb protects you from attacks, scents, and insulates you "
       "from electricity. It does not let you go through deadly terrain, but "
       "if you are attacked with fire, it lets you stay in place in it.");
  if(it == itOrbEmpathy) {
    int cnt = 0;
    for(int i=0; i<ittypes; i++) {
      eItem it2 = eItem(i);
      if(isEmpathyOrb(it2)) {
        help += XLAT(cnt ? ", %1" : " %1", it2);
        cnt++;
        }
      }
    }
  
  if(inv::on) {
    if(it == itOrbYendor || it == itHell) {
      help += XLAT(
        "\n\nIn the Orb Strategy Mode, Orbs of Yendor appear in Hell after "
        "you collect 25 Demon Daisies in Hell, in Crossroads/Ocean after you collect 50, "
        "and everywhere after you collect 100.");  
      }
    
/*    if(it == itBone || it == itGreenStone) {
      help += XLAT(
        "\n\nIn the Orb Strategy Mode, dead orbs are available once you collect "
        "10 Necromancer Totems in the Graveyard."
        );
      } */
    
    if(it == itFeather || it == itOrbSafety) {
      help += XLAT(
        "\n\nIn the Orb Strategy Mode, Orbs of Safety can be gained by "
        "collecting Phoenix Feathers in the Land of Eternal Motion. "
        "You can also find unlimited Orbs of Safety in the Crossroads "
        "and the Ocean (after collecting 25 Phoenix Feathers) "
        "and in the Prairie."
        );
      }
  
    if(it == itOrbYendor || it == itHolyGrail)
      help += XLAT(
        "\n\nCollect %the1 to gain an extra Orb of the Mirror. "
        "You can gain further Orbs of the Mirror by collecting 2, 4, 8..."
        );  
    
    if(it == itPower)
      help += XLAT(
        "\n\nThe amount of Orbs obtained by using Orbs of Mirroring is "
        "multiplied by sqrt(1+p/20), where p is the number of Powerstones "
        "collected. This also affects the mirrorings which happened before "
        "collecting the Powerstones."
        );
    
    if(it == itOrbLuck)
      help += XLAT(
        "\n\nIn the Orb Strategy Mode, the Orb of Luck also "
        "significantly increases the frequency of Great Walls, Crossroads IV, "
        "and sub-lands."
        );

    if(it == itBone)
      help += XLAT(
        "\n\nIn the Orb Strategy Mode, each 25 Necromancer's Totems "
        "you are given a random offensive Orb."
        );
    }
  
  if(itemclass(it) == IC_ORB || it == itGreenStone || it == itOrbYendor) {
    for(int i=0; i<ORBLINES; i++) {
      const orbinfo& oi(orbinfos[i]);
      if(oi.orb == it) describeOrb(help, oi);
      }
    }
  
  if(itemclass(it) == IC_TREASURE) {
    for(int i=0; i<ORBLINES; i++) {
      const orbinfo& oi(orbinfos[i]);
      if(treasureType(oi.l) == it) {
        if(oi.gchance > 0) {
          help += XLAT("\n\nOrb unlocked: %1", oi.orb);
          describeOrb(help, oi);
          }
        else if(oi.l == cwt.c->land || inv::on) {
          help += XLAT("\n\nSecondary orb: %1", oi.orb);
          describeOrb(help, oi);
          }
        }
      }
    }

  return help;
  }

void addMinefieldExplanation(string& s) {

  s += XLAT(
    "\n\nOnce you collect 10 Bomberbird Eggs, "
    "stepping on a cell with no adjacent mines also reveals the adjacent cells. "
    "Collecting even more Eggs will increase the radius. Additionally, collecting "
    "25 Bomberbird Eggs will reveal adjacent cells even in your future games."
    );

  s += "\n\n";
#if ISMOBILE==1
  s += XLAT("Known mines may be marked by pressing 'm'. Your allies won't step on marked mines.");
#else
  s += XLAT("Known mines may be marked by touching while in drag mode. Your allies won't step on marked mines.");
#endif
  }

string generateHelpForWall(eWall w) {

  string s = helptitle(XLATN(winf[w].name), winf[w].color);
   
  s += XLAT(winf[w].help);
  if(w == waMineMine || w == waMineUnknown || w == waMineOpen)
    addMinefieldExplanation(s);
  if(isThumper(w)) s += pushtext(w);
  if((w == waClosePlate || w == waOpenPlate) && purehepta) 
    s += "\n\n(For the heptagonal mode, the radius has been reduced to 2 for closing plates.)";
  return s;
  }

void buteol(string& s, int current, int req) {
  int siz = size(s);
  if(s[siz-1] == '\n') s.resize(siz-1);
  char buf[100]; sprintf(buf, " (%d/%d)", current, req);
  s += buf; s += "\n";
  }

string generateHelpForMonster(eMonster m) {
  string s = helptitle(XLATN(minf[m].name), minf[m].color);
  
  if(m == moPlayer) {
#if CAP_TOUR
    if(tour::on || peace::on)
      return s+XLAT(
        "A tourist from another world. They mutter something about the 'tutorial', "
        "and claim that they are here just to learn, and to leave without any treasures. "
        "Do not kill them!"
        );
#endif

    s += XLAT(
        "This monster has come from another world, presumably to steal our treasures. "
        "Not as fast as an Eagle, not as resilient as the guards from the Palace, "
        "and not as huge as the Mutant Ivy from the Clearing; however, "
        "they are very dangerous because of their intelligence, "
        "and access to magical powers.\n\n");
    
    if(cheater)
      s += XLAT("Actually, their powers appear god-like...\n\n");
    
    else if(!hardcore)
      s += XLAT(
       "Rogues will never make moves which result in their immediate death. "
       "Even when cornered, they are able to instantly teleport back to their "
       "home world at any moment, taking the treasures forever... but "
       "at least they will not steal anything further!\n\n"
       );
    
    if(!euclid)
      s += XLAT(
        "Despite this intelligence, Rogues appear extremely surprised "
        "by the most basic facts about geometry. They must come from "
        "some really strange world.\n\n"
        );
    
    if(shmup::on)
      s += XLAT("In the Shoot'em Up mode, you are armed with thrown Knives.");
    
    return s;
    }

  s += XLAT(minf[m].help);      
  if(m == moPalace || m == moSkeleton)
    s += pushtext(m);  
  if(m == moTroll) s += XLAT(trollhelp2);  

  if(isMonsterPart(m))
    s += XLAT("\n\nThis is a part of a monster. It does not count for your total kills.", m);

  if(isFriendly(m))
    s += XLAT("\n\nThis is a friendly being. It does not count for your total kills.", m);

  if(m == moTortoise)
    s += XLAT("\n\nTortoises are not monsters! They are just annoyed. They do not count for your total kills.", m);
  
  if(isGhost(m))
    s += XLAT("\n\nA Ghost never moves to a cell which is adjacent to another Ghost of the same kind.", m);
    
  if(m == moBat || m == moEagle)
    s += XLAT("\n\nFast flying creatures may attack or go against gravity only in their first move.", m);

  return s;
  }

string generateHelpForLand(eLand l) {
  string s = helptitle(XLATN(linf[l].name), linf[l].color);
  
  if(l == laPalace) s += princedesc();

  s += XLAT(linf[l].help);

  if(l == laMinefield) addMinefieldExplanation(s);

  s += "\n\n";
  if(l == laIce || l == laCaves || l == laDesert || l == laMotion || l == laJungle ||
    l == laCrossroads || l == laAlchemist)
      s += XLAT("Always available.\n");

  #define ACCONLY(z) s += XLAT("Accessible only from %the1.\n", z);
  #define ACCONLY2(z,x) s += XLAT("Accessible only from %the1 or %the2.\n", z, x);
  #define ACCONLYF(z) s += XLAT("Accessible only from %the1 (until finished).\n", z);
  #define TREQ(z) { s += XLAT("Treasure required: %1 $$$.\n", its(z)); buteol(s, gold(), z); }
  #define TREQ2(z,x) { s += XLAT("Treasure required: %1 x %2.\n", its(z), x); buteol(s, items[x], z); }
  
  if(l == laMirror || l == laMinefield || l == laPalace ||
    l == laOcean || l == laLivefjord || l == laZebra || l == laWarpCoast || l == laWarpSea ||
    l == laReptile || l == laIvoryTower)
      TREQ(R_30)

  if(l == laPower && inv::on)
    help += XLAT(
      "\n\nThe amount of Orbs obtained by using Orbs of Mirroring is "
      "multiplied by sqrt(1+p/20), where p is the number of Powerstones "
      "collected. This also affects the mirrorings which happened before "
      "collecting the Powerstones."
      );
  
  if(isCoastal(l)) 
    s += XLAT("Coastal region -- connects inland and aquatic regions.\n");
    
  if(isPureSealand(l)) 
    s += XLAT("Aquatic region -- accessible only from coastal regions and other aquatic regions.\n");
    
  if(l == laWhirlpool) ACCONLY(laOcean)
  if(l == laRlyeh) ACCONLYF(laOcean)
  if(l == laTemple) ACCONLY(laRlyeh)  
  if(l == laClearing) ACCONLY(laOvergrown)  
  if(l == laHaunted) ACCONLY(laGraveyard)  
  if(l == laPrincessQuest) ACCONLY(laPalace)
  if(l == laMountain) ACCONLY(laJungle)
  if(l == laCamelot) ACCONLY2(laCrossroads, laCrossroads3)
  
  if(l == laDryForest || l == laWineyard || l == laDeadCaves || l == laHive || l == laRedRock ||
    l == laOvergrown || l == laStorms || l == laWhirlwind || l == laRose ||
    l == laCrossroads2 || l == laRlyeh)
      TREQ(R_60)
    
  if(l == laReptile) TREQ2(U_10, itElixir)
  if(l == laEndorian) TREQ2(U_10, itIvory)
  if(l == laKraken) TREQ2(U_10, itFjord)
  if(l == laBurial) TREQ2(U_10, itKraken)

  if(l == laDungeon) TREQ2(U_5, itIvory)
  if(l == laDungeon) TREQ2(U_5, itPalace)
  if(l == laMountain) TREQ2(U_5, itIvory)
  if(l == laMountain) TREQ2(U_5, itRuby)
    
  if(l == laPrairie) TREQ(R_90)
  if(l == laBull) TREQ(R_90)
  if(l == laCrossroads4) TREQ(R_200)
  if(l == laCrossroads5) TREQ(R_300)
  
  if(l == laGraveyard || l == laHive) {
    s += XLAT("Kills required: %1.\n", "100");
    buteol(s, tkills(), R_100);
    }
  
  if(l == laDragon) {
    s += XLAT("Different kills required: %1.\n", "20");
    buteol(s, killtypes(), R_20);
    }
  
  if(l == laTortoise) ACCONLY(laDragon)
  if(l == laTortoise) s += XLAT("Find a %1 in %the2.", itBabyTortoise, laDragon);

  if(l == laHell || l == laCrossroads3) {
    s += XLAT("Finished lands required: %1 (collect %2 treasure)\n", "9", its(R_10));
    buteol(s, orbsUnlocked(), 9);
    }
  
  if(l == laCocytus || l == laPower) TREQ2(U_10, itHell)
  if(l == laRedRock) TREQ2(U_10, itSpice)
  if(l == laOvergrown) TREQ2(U_10, itRuby)
  if(l == laClearing) TREQ2(U_5, itMutant)
  if(l == laCocytus) TREQ2(U_10, itDiamond)
  if(l == laDeadCaves) TREQ2(U_10, itGold)
  if(l == laTemple) TREQ2(U_5, itStatue)
  if(l == laHaunted) TREQ2(U_10, itBone)
  if(l == laCamelot) TREQ2(U_5, itEmerald)
  if(l == laEmerald) {
    TREQ2(U_5, itFernFlower) TREQ2(U_5, itGold)
    s += XLAT("Alternatively: kill a %1 in %the2.\n", moVizier, laPalace);
    buteol(s, kills[moVizier], 1);
    }
  
#define KILLREQ(who, where) { s += XLAT("Kills required: %1 (%2).\n", who, where); buteol(s, kills[who], 1); }

  if(l == laPrincessQuest)
    KILLREQ(moVizier, laPalace);
    
  if(l == laElementalWall) {
    KILLREQ(moFireElemental, laDragon);
    KILLREQ(moEarthElemental, laDeadCaves);
    KILLREQ(moWaterElemental, laLivefjord);
    KILLREQ(moAirElemental, laWhirlwind);
    }
  
  if(l == laTrollheim) {
    KILLREQ(moTroll, laCaves);
    KILLREQ(moFjordTroll, laLivefjord);
    KILLREQ(moDarkTroll, laDeadCaves);
    KILLREQ(moStormTroll, laStorms);
    KILLREQ(moForestTroll, laOvergrown);
    KILLREQ(moRedTroll, laRedRock);
    }
  
  if(l == laZebra) TREQ2(U_10, itFeather)
  
  if(l == laCamelot || l == laPrincessQuest)
    s += XLAT("Completing the quest in this land is not necessary for the Hyperstone Quest.");

  int rl = isRandland(l);
  if(rl == 2)
    s += XLAT("Variants of %the1 are always available in the Random Pattern Mode.", l);
  else if(rl == 1)
    s += XLAT(
      "Variants of %the1 are available in the Random Pattern Mode after "
      "getting a highscore of at least 10 %2.", l, treasureType(l));

  if(l == laPrincessQuest) {
    s += XLAT("Unavailable in the shmup mode.\n");
    s += XLAT("Unavailable in the multiplayer mode.\n");
    }
  
  if(noChaos(l))
    s += XLAT("Unavailable in the Chaos mode.\n");
  
  if(l == laWildWest)
    s += XLAT("Bonus land, available only in some special modes.\n");
  
  if(l == laWhirlpool)
    s += XLAT("Orbs of Safety always appear here, and may be used to escape.\n");

  /* if(isHaunted(l) || l == laDungeon)
    s += XLAT("You may be unable to leave %the1 if you are not careful!\n", l); */
    
  if(l == laStorms) {
    if(elec::lightningfast == 0) s += XLAT("\nSpecial conduct (still valid)\n");
    else s += XLAT("\nSpecial conduct failed:\n");
    
    s += XLAT(
      "Avoid escaping from a discharge (\"That was close\").");
    }

  if(isHaunted(l)) {
    if(survivalist) s += XLAT("\nSpecial conduct (still valid)\n");
    else s += XLAT("\nSpecial conduct failed:\n");

    s += XLAT(
      "Avoid chopping trees, using Orbs, and non-graveyard monsters in the Haunted Woods."
      );
    }
  
#if !ISMOBILE
  if(l == laCA)
    s += XLAT("\n\nHint: use 'm' to toggle cells quickly");
#endif

  return s;
  }

bool instat;

string turnstring(int i) {
  if(i == 1) return XLAT("1 turn");
  else return XLAT("%1 turns", its(i));
  }

void describeMouseover() {
  DEBB(DF_GRAPH, (debugfile,"describeMouseover\n"));

  cell *c = mousing ? mouseover : playermoved ? NULL : centerover;
  string& out = mouseovers;
  if(!c || instat || getcstat != '-') { }
  else if(c->wall != waInvisibleFloor) {
    out = XLAT1(linf[c->land].name);
    help = generateHelpForLand(c->land);
    
    if(c->land == laIce || c->land == laCocytus) 
      out += " (" + fts(heat::celsius(c)) + " °C)";
    if(c->land == laDryForest && c->landparam) 
      out += " (" + its(c->landparam)+"/10)";
    if(c->land == laOcean && chaosmode)
      out += " (" + its(c->CHAOSPARAM)+"S"+its(c->SEADIST)+"L"+its(c->LANDDIST)+")";
    else if(c->land == laOcean && c->landparam <= 25) {
      if(shmup::on)
        out += " (" + its(c->landparam)+")";
      else {
        bool b = c->landparam >= tide[(turncount-1) % tidalsize];
        int t = 1;
        for(; t < 1000 && b == (c->landparam >= tide[(turncount+t-1) % tidalsize]); t++) ;
        if(b)
          out += " (" + turnstring(t) + XLAT(" to surface") + ")";
        else 
          out += " (" + turnstring(t) + XLAT(" to submerge") + ")";
        }
      }

    if(c->land == laTortoise && tortoise::seek()) out += " " + tortoise::measure(getBits(c));

    if(buggyGeneration) {
      char buf[80]; sprintf(buf, " %p H=%d M=%d", c, c->landparam, c->mpdist); out += buf;
      }
    
    if(randomPatternsMode)
      out += " " + describeRPM(c->land);
      
    if(euclid && cheater) {
      if(torus) {
        out += " ("+its(decodeId(c->master))+")";
        }
      else {
        eucoord x, y;
        decodeMaster(c->master, x, y);
        out += " ("+its(short(x))+","+its(short(y))+")";
        }
      }
      
    if(c->wall && 
      !((c->wall == waFloorA || c->wall == waFloorB || c->wall == waFloorC || c->wall == waFloorD) && c->item)) { 
      out += ", "; out += XLAT1(winf[c->wall].name); 
      
      if(c->wall == waRose) out += " (" + its(7-rosephase) + ")";
      
      if((c->wall == waBigTree || c->wall == waSmallTree) && c->land != laDryForest)
        help = 
          "Trees in this forest can be chopped down. Big trees take two turns to chop down.";
      else if(c->wall != waSea && c->wall != waPalace)
      if(!((c->wall == waCavefloor || c->wall == waCavewall) && c->land == laEmerald))
        help = generateHelpForWall(c->wall);
      }
    
    if(isActivable(c)) out += XLAT(" (touch to activate)");
    
    if(hasTimeout(c)) out += " [" + turnstring(c->wparam) + "]";
    
    if(isReptile(c->wall))
      out += " [" + turnstring((unsigned char) c->wparam) + "]";
  
    if(c->monst) {
      out += ", "; out += XLAT1(minf[c->monst].name); 
      if(hasHitpoints(c->monst))
        out += " (" + its(c->hitpoints)+" HP)";
      if(isMutantIvy(c))
        out += " (" + its((c->stuntime - mutantphase) & 15) + "*)";
      else if(c->stuntime)
        out += " (" + its(c->stuntime) + "*)";

      if(c->monst == moTortoise && tortoise::seek()) 
        out += " " + tortoise::measure(tortoise::getb(c));

      help = generateHelpForMonster(c->monst);
      }
  
    if(c->item && !itemHiddenFromSight(c)) {
      out += ", "; 
      out += XLAT1(iinf[c->item].name); 
      if(c->item == itBarrow) out += " (x" + its(c->landparam) + ")";
      if(c->item == itBabyTortoise && tortoise::seek()) 
        out += " " + tortoise::measure(tortoise::babymap[c]);
      if(!c->monst) help = generateHelpForItem(c->item);
      }
    
    if(isPlayerOn(c) && !shmup::on) out += XLAT(", you"), help = generateHelpForMonster(moPlayer);

    shmup::addShmupHelp(out);

    if(rosedist(c) == 1)
      out += ", wave of scent (front)";

    if(rosedist(c) == 2)
      out += ", wave of scent (back)";
    
    if(sword::at(c)) out += ", Energy Sword";
    
    if(rosedist(c) || c->land == laRose || c->wall == waRose)
      help += s0 + "\n\n" + rosedesc;
    
    if(isWarped(c) && !isWarped(c->land))
      out += ", warped";

    if(isWarped(c)) 
      help += s0 + "\n\n" + warpdesc;
    }
    
#if CAP_ROGUEVIZ
  rogueviz::describe(c);
#endif
  
  if(mousey < vid.fsize * 3/2) getcstat = SDLK_F1;
  }

void showHelp() {
  gamescreen(2);
  cmode = sm::HELP | sm::DOTOUR;
  getcstat = SDLK_ESCAPE;
  if(help == "HELPFUN") {
    help_delegate();
    return;
    }

  if(help == "@") help = buildHelpText();
  
  string help2;
  if(help[0] == '@') {
    int iv = help.find("\t");
    int id = help.find("\n");
    dialog::init(help.substr(iv+1, id-iv-1), atoi(help.c_str()+1), 120, 100);
    dialog::addHelp(help.substr(id+1));
    }
  else {
    dialog::init("help", forecolor, 120, 100);
    dialog::addHelp(help);
    }
  
  if(help == buildHelpText()) dialog::addItem("credits", 'c');
  
  dialog::display();
  
  keyhandler = [] (int sym, int uni) {
    dialog::handleNavigation(sym, uni);
    if(sym == SDLK_F1 && help != "@") 
      help = "@";
    else if(uni == 'c')
      help = buildCredits();
    else if(doexiton(sym, uni))
      popScreen();
    };
  }

void gotoHelp(const string& h) {
  help = h;
  pushScreen(showHelp);
  }
