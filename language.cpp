// Hyperbolic Rogue -- language support
// Copyright (C) 2011-2019 Zeno Rogue, see 'hyper.cpp' for details

/** \file language.cpp
 *  \brief localization support
 */

// #define CHECKTRANS

#include "hyper.h"
namespace hr {

EX string dnameof(eMonster m) { return m >= 0 && m < motypes ? minf[m].name : hr::format("[MONSTER %d]", m); }
EX string dnameof(eLand l) { return l >= 0 && l < landtypes ? linf[l].name : hr::format("[LAND %d]", l); }
EX string dnameof(eWall w) { return w >= 0 && w < walltypes ? winf[w].name : hr::format("[WALL %d]", w); }
EX string dnameof(eItem i) { return i >= 0 && i < ittypes ? iinf[i].name : hr::format("[ITEM %d]", i); }

#if HDR
#define NUMLAN 9

struct stringpar {
  string v;
  stringpar(string s) : v(s) { }
  stringpar(const char* s) : v(s) { }
  stringpar(eMonster m) { v= minf[m].name; }
  stringpar(eLand l) { v= linf[l].name; }
  stringpar(eWall w) { v= winf[w].name; }
  stringpar(eItem i) { v= iinf[i].name; }  
  };
#endif

void rep(string& pattern, string what, string to) {
  while(true) {
    size_t at = pattern.find(what);
    if(at == string::npos) break;
      pattern = pattern.replace(at, what.size(), to);
    }
  }

void reponce(string& pattern, string what, string to) {
  size_t at = pattern.find(what);
  if(at != string::npos)
    pattern = pattern.replace(at, what.size(), to);
  }

typedef unsigned hashcode;

struct sentence {
  hashcode langhash;
  const char* xlat[NUMLAN-1];
  };

struct noun {
  int genus;
  const char *nom, *nomp, *acc, *abl;
  };

struct fullnoun {
  hashcode langhash;
  int english_grammar_flags;
  noun n[NUMLAN-1];
  };

#if !CAP_TRANS
#if HDR
#define NUMEXTRA 13
extern const char* natchars[NUMEXTRA];
#endif
const char* natchars[NUMEXTRA] = {"°","é","á", "²", "½", "Θ", "δ", "π", "ϕ", "ᵈ", "∞", "⌫", "√"};
#endif

#if CAP_TRANS
#include "language-data.cpp"

hashcode langhash(const string& s) {
  hashcode r = 0;
  for(int i=0; i<isize(s); i++) r = hashval * r + s[i];
  return r;
  }

template<class T> const T* findInHashTableS(string s, const T *table, int size) {
  int b = 0, e = size;
  hashcode h = langhash(s);
  while(b!=e) {
    int m = (b+e)>>1;
    // printf("b=%d e=%d m=%d h=%x s=%x\n", b, e, m, table[m].langhash, h);
    if(table[m].langhash >= h) e = m;
    else b = m+1;
    }
  if(e != size && table[e].langhash == h)
    return &table[e];
  return NULL;
  }

#define findInHashTable(s,t) findInHashTableS(s, t, sizeof(t) / sizeof(t[0]))
#endif

string choose2(int g, string a, string b) {
  g &= GEN_BASE_MASK;
  if(g == GEN_M || g == GEN_O) return a;
  if(g == GEN_F || g == GEN_N) return b;
  return "?" + a;
  }

string choose3(int g, string a, string b, string c) {
  g &= GEN_BASE_MASK;
  if(g == GEN_M || g == GEN_O) return a;
  if(g == GEN_F) return b;
  if(g == GEN_N) return c;
  return "?" + a;
  }

string choose4(int g, string a, string b, string c, string d) {
  g &= GEN_BASE_MASK;
  if(g == GEN_M) return a;
  if(g == GEN_F) return b;
  if(g == GEN_N) return c;
  if(g == GEN_O) return d;
  return "?" + a;
  }

set<string> warnshown;

#if CAP_TRANS
EX bool translation_exists(const string& x) {
  return findInHashTable(x, all_sentences);
  }
#endif

/** replace gender-based codes in x, based on gender genus; some gender-based codes need to know the word (nom) */
void genderrep(string& x, const string& w, const noun& N) {
#if CAP_TRANS
  int l = lang();
  auto& genus = N.genus;
  if(l == 1) {
    rep(x, "%łem"+w, choose3(genus, "łem", "łam", "łom"));
    rep(x, "%łeś"+w, choose3(genus, "łeś", "łaś", "łoś"));
    rep(x, "%łeś"+w, choose3(genus, "łeś", "łaś", "łoś"));
    rep(x, "%ął"+w, choose3(genus, "ął", "ęła", "ęło"));
    rep(x, "%ąłeś"+w, choose3(genus, "ąłeś", "ęłaś", "ęłoś"));
    rep(x, "%ógł"+w, choose3(genus, "ógł", "ogła", "ogło"));
    rep(x, "%ł"+w, choose3(genus, "ł", "ła", "ło"));
    rep(x, "%y"+w, choose3(genus, "y", "a", "e"));
    rep(x, "%ya"+w, choose3(genus, "y", "a", "e"));
    rep(x, "%yą"+w, choose4(genus, "ego", "ą", "e", "y"));
    rep(x, "%oa"+w, choose3(genus, "", "a", "o"));
    rep(x, "%ymą"+w, choose3(genus, "ym", "ą", "ym"));
    rep(x, "%go"+w, choose3(genus, "go", "ją", "je"));
    rep(x, "%aka"+w, choose3(genus, "a", "ka", "a"));
    }

  if(l == 3) {
    rep(x, "%l"+w, choose3(genus, "l", "la", "lo"));
    rep(x, "%d"+w, choose3(genus, "", "a", "o"));
    rep(x, "%ý"+w, choose3(genus, "ý", "á", "é"));
    rep(x, "%el"+w, choose3(genus, "el", "la", "lo"));
    rep(x, "%ůj"+w, choose4(genus, "ého", "ou", "é", "ůj"));
    rep(x, "%ým"+w, choose3(genus, "ým", "ou", "ým"));
    rep(x, "%ho"+w, choose3(genus, "ho", "ji", "ho"));
    rep(x, "%ého"+w, choose3(genus, "ého", "ou", "ého"));
    }

  if(l == 4) {
    rep(x, "%E"+w, choose3(genus, "", "а", "о"));
    rep(x, "%A"+w, choose3(genus, "ый", "ая", "ое"));
    rep(x, "%c"+w, choose3(genus, "ся", "ась", ""));
    rep(x, "%y"+w, choose3(genus, "ый", "ая", "ое"));
    }

  if(l == 5) {
    rep(x, "%Der"+w, genus == -1 ? "The" : choose3(genus, "Der", "Die", "Das"));
    rep(x, "%der"+w, genus == -1 ? "the" : choose3(genus, "der", "die", "das"));
    rep(x, "%den"+w, genus == -1 ? "the" : choose3(genus, "den", "die", "das"));
    rep(x, "%dem"+w, genus == -1 ? "the" : choose3(genus, "dem", "der", "dem"));
    }

  if(l == 6) {
    rep(x, "%oa"+w, choose4(genus, "o", "a", "os", "as"));
    rep(x, "%seu"+w, choose4(genus, "seu", "sua", "seus", "suas"));
    rep(x, "%na"+w, choose4(genus, "o", "a", "os", "as") + " " + N.nom);
    rep(x, "%Na"+w, choose4(genus, "O", "A", "Os", "As") + " " + N.nom);
    rep(x, "%g"+w, choose4(genus, "do", "da", "dos", "das")+ " " + N.nom);
    rep(x, "%d"+w, choose4(genus, "ao", "à", "aos", "às")+ " " + N.nom);
    rep(x, "%l"+w, choose4(genus, "no", "na", "nos", "nas")+ " " + N.nom);
    rep(x, "%abl"+w, choose4(genus, "pelo", "pela", "pelos", "pelas")+ " " + N.nom);
    }

  if(l == 7) {
    if(genus & GENF_PROPER)
    {
      rep(x, "%le"+w, N.nom);
      rep(x, "%Le"+w, N.nom);
      rep(x, "%un"+w, N.nom);
      rep(x, "%Un"+w, N.nom);
      }
    else if(genus & GENF_PLURALONLY)
    {
      rep(x, "%le"+w, s0+"les "+N.nomp);
      rep(x, "%Le"+w, s0+"Les "+N.nomp);
      rep(x, "%un"+w, s0+"des "+N.nomp);
      rep(x, "%Un"+w, s0+"Des "+N.nomp);
      }
    else if(genus & GENF_ELISION)
    {
      rep(x, "%de"+w, s0+"d'"+N.nom);
      rep(x, "%le"+w, s0+"l'"+N.nom);
      rep(x, "%Le"+w, s0+"L'"+N.nom);
      }
    if(genus == 0) {
      rep(x, " de %le"+w, s0+" du "+N.nom);
      rep(x, " à %le"+w, s0+" au "+N.nom);
      }
    rep(x, "%le"+w, choose2(genus, "le ", "la ")+N.nom);
    rep(x, "%Le"+w, choose2(genus, "Le ", "La ")+N.nom);
    rep(x, "%un"+w, choose2(genus, "un ", "une ")+N.nom);
    rep(x, "%Un"+w, choose2(genus, "Un ", "Une ")+N.nom);
    rep(x, "%de"+w, s0+"de "+N.nom);
    
    rep(x, "%er"+w, choose2(genus, "er", "ère"));
    rep(x, "%e"+w, choose2(genus, "", "e"));
    rep(x, "%x"+w, choose2(genus, "x", "se"));
    }
#endif
  }

void basicrep(string& x) {
#if CAP_TRANS
  const sentence *s = findInHashTable(x, all_sentences);
  if(!s && !warnshown.count(x)) {
    printf("WARNING: no translations for '%s'\n", x.c_str());
    warnshown.insert(x);
    }

  int l = lang();
  if(l) {
    const sentence *s = findInHashTable(x, all_sentences);
    if(s) x = s->xlat[l-1];
    }
  
  noun dummy;
  dummy.genus = playergender();
  dummy.nom = dummy.nomp = dummy.acc = dummy.abl = "player";
  genderrep(x, "0", dummy);
#endif
  }

void parrep(string& x, string w, stringpar p) {
#if CAP_TRANS
  int l = lang();
  const fullnoun *N = findInHashTable(p.v, all_nouns);
  noun dummy;
  auto &data = N ? N->n[l-1] : dummy;
  if(!N) {
    dummy.nom = dummy.nomp = dummy.acc = dummy.abl = p.v.c_str();
    dummy.genus = -1;
    }
  
  genderrep(x, w, data);

  if(l == 1) {
    rep(x, "%"+w, data.nom);
    rep(x, "%P"+w, data.nomp);
    rep(x, "%a"+w, data.acc);
    rep(x, "%abl"+w, data.abl);
    }
  if(l == 2) {
    rep(x, "%"+w, N->n[1].nom);
    rep(x, "%P"+w, N->n[1].nomp);
    rep(x, "%a"+w, N->n[1].acc);
    rep(x, "%abl"+w, N->n[1].abl);
    }
  if(l == 3) {
    rep(x, "%"+w, data.nom);
    rep(x, "%P"+w, data.nomp);
    rep(x, "%a"+w, data.acc);
    rep(x, "%abl"+w, data.abl);
    if(p.v == "Mirror Image")
      rep(x, "%s"+w, "se");
    if(p.v == "Mirage")
      rep(x, "%s"+w, "s");
    }
  if(l == 4) {
    rep(x, "%"+w, data.nom);
    rep(x, "%P"+w, data.nomp);
    rep(x, "%a"+w, data.acc);
    rep(x, "%abl"+w, data.abl);
    }
  if(l == 5) {
    rep(x, "%"+w, data.nom);
    rep(x, "%P"+w, data.nomp);
    rep(x, "%a"+w, data.acc);
    rep(x, "%abl"+w, data.abl);
    rep(x, "%d"+w, data.abl); // Dativ (which equals Ablative in German)
    }
  if(l == 6) {
    rep(x, "%"+w, data.nom);
    rep(x, "%P"+w, data.nomp);
    }
  if(l == 7) {
    rep(x, "%"+w, data.nom);
    rep(x, "%P"+w, data.nomp);
    rep(x, "%a"+w, data.acc);
    rep(x, "%abl"+w, data.abl);
    rep(x, "%d"+w, data.abl); // Dativ (which equals Ablative in German)
    }
  if(l == 8) {
    rep(x, "%"+w, data.nom);
    rep(x, "%m"+w, data.abl); // Measure word in Chinese
    }
#endif
  if(true) {
    // proper names (R'Lyeh)
    rep(x,"%"+w,p.v);
#if !CAP_TRANS
    int flags = 0;
    if(p.v == "R'Lyeh" || p.v == "Camelot")  flags = 1;
    if(p.v == "Crossroads" || p.v == "Crossroads II" ||
      p.v == "Crossroads III" || p.v == "Crossroads IV" ||
      p.v == "Kraken Depths" || p.v == "Elemental Planes")
      flags = 2;
#else
    int flags = N ? N->english_grammar_flags : 0;
#endif
    if(flags & 1) {
      rep(x,"%the"+w, p.v);
      rep(x,"%The"+w, p.v);
      }
    else {
      rep(x,"%the"+w, "the " + p.v);
      rep(x,"%The"+w, "The " + p.v);
      rep(x,"%him"+w, princessgender() ? "her" : "him");
      rep(x,"%his"+w, princessgender() ? "her" : "his");
      }
    // plural names (Crossroads)
    if(flags & 2)
      rep(x,"%s"+w, "");
    else 
      rep(x,"%s"+w, "s");
    }
  return;
  }

void postrep(string& s) {
  }

/** translate the string @x */
EX string XLAT(string x) { 
  basicrep(x);
  postrep(x);
  return x;
  }
EX string XLAT(string x, stringpar p1) { 
  basicrep(x);
  parrep(x,"1",p1.v);
  postrep(x);
  return x;
  }
EX string XLAT(string x, stringpar p1, stringpar p2) { 
  basicrep(x);
  parrep(x,"1",p1.v);
  parrep(x,"2",p2.v);
  postrep(x);
  return x;
  }
EX string XLAT(string x, stringpar p1, stringpar p2, stringpar p3) { 
  basicrep(x);
  parrep(x,"1",p1.v);
  parrep(x,"2",p2.v);
  parrep(x,"3",p3.v);
  postrep(x);
  return x;
  }
EX string XLAT(string x, stringpar p1, stringpar p2, stringpar p3, stringpar p4) { 
  basicrep(x);
  parrep(x,"1",p1.v);
  parrep(x,"2",p2.v);
  parrep(x,"3",p3.v);
  parrep(x,"4",p4.v);
  postrep(x);
  return x;
  }
EX string XLAT(string x, stringpar p1, stringpar p2, stringpar p3, stringpar p4, stringpar p5) { 
  basicrep(x);
  parrep(x,"1",p1.v);
  parrep(x,"2",p2.v);
  parrep(x,"3",p3.v);
  parrep(x,"4",p4.v);
  parrep(x,"5",p5.v);
  postrep(x);
  return x;
  }

EX const char* XLAT1_to(string x, int language) {
#if CAP_TRANS
  const fullnoun *N = findInHashTable(x, all_nouns);
  if(N) return N->n[7].nom;
#endif
  return nullptr;
  }

EX const char* XLAT1_acc(string x, int language) {
#if CAP_TRANS
  const fullnoun *N = findInHashTable(x, all_nouns);
  if(N) return N->n[7].acc;
#endif
  return nullptr;
  }

EX string XLATN(string x) { 
#if CAP_TRANS
  if(lang()) {
    const fullnoun *N = findInHashTable(x, all_nouns);
    if(N) return N->n[lang()-1].nomp;
    }
#endif
  return x;
  }

EX string XLAT1(string x) {
#if CAP_TRANS
  if(lang()) {
    const fullnoun *N = findInHashTable(x, all_nouns);
    if(N) return N->n[lang()-1].nom;
    }
#endif
  return x;
  }

EX string XLATT1(stringpar p) { return XLAT1(p.v); }

EX bool shapename_available(int id) { return id >= 3 && id < 9; }

EX string shapename(int id) {
  if(!shapename_available(id)) return "";
  string english[9] = {"", "", "digon", "triangle", "square", "pentagon", "hexagon", "heptagon", "octagon"};
  return english[id];
  }

}
