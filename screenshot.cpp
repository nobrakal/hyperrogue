// Hyperbolic Rogue -- screenshots in SVG and PNG formats
// Copyright (C) 2011-2018 Zeno Rogue, see 'hyper.cpp' for details

namespace hr {

#if ISMOBILE==1
namespace svg {
  bool in = false;
  }
#endif

#if ISMOBILE==0
// svg renderer
namespace svg {
  #if ISWEB
  shstream f;
  #else
  fhstream f;
  #endif
  
  bool in = false;
  
  ld cta(color_t col) {
    // col >>= 24;
    col &= 0xFF;
    return col / 255.0;
    }
  
  bool invisible(color_t col) { return (col & 0xFF) == 0; }
  
  void fixgamma(unsigned int& color) {
    unsigned char *c = (unsigned char*) (&color);
    for(int i=1; i<4; i++) c[i] = 255 * pow(float(c[i] / 255.0), float(shot::gamma));
    }
  
  int svgsize;
  int divby = 10;
  
  const char* coord(int val) {
    static char buf[10][20];
    static int id;
    id++; id %= 10;
    if(divby == 1) {
      sprintf(buf[id], "%d", val); return buf[id];
      }
    else if(divby <= 10) {
      sprintf(buf[id], "%.1f", val*1./divby); return buf[id];
      }
    else {
      sprintf(buf[id], "%.2f", val*1./divby); return buf[id];
      }
    }
  
  char* stylestr(unsigned int fill, unsigned int stroke, ld width=1) {
    fixgamma(fill);
    fixgamma(stroke);
    static char buf[600];
    // printf("fill = %08X stroke = %08x\n", fill, stroke);
  
    if(stroke == 0xFF00FF && false) {
      stroke = 0x000000FF;
      
      if(fill == 0x332a22ff) fill = 0x000000FF;
      else if(fill == 0x686868FF) fill = 0x000000FF;
      else if(fill == 0xd0d0d0FF) fill = 0x000000FF;
      else fill = 0xFFFFFFFF;
      }
    
    sprintf(buf, "style=\"stroke:#%06x;stroke-opacity:%.3" PLDF ";stroke-width:%" PLDF "px;fill:#%06x;fill-opacity:%.3" PLDF "\"",
      (stroke>>8) & 0xFFFFFF, cta(stroke),
      width/divby,
      (fill>>8) & 0xFFFFFF, cta(fill)
      );
    return buf;
    }
  
  void circle(int x, int y, int size, color_t col, color_t fillcol, double linewidth) {
    if(!invisible(col) || !invisible(fillcol)) {
      if(vid.stretch == 1)
        println(f, "<circle cx='", coord(x), "' cy='", coord(y), "' r='", coord(size), "' ", stylestr(fillcol, col, linewidth), "/>");
      else
        println(f, "<ellipse cx='", coord(x), "' cy='", coord(y), "' rx='", coord(size), "' ry='", coord(size*vid.stretch), "' ", stylestr(fillcol, col), "/>");
      }
    }
  
  string link;
  
  void startstring() {
    if(link != "") print(f, "<a xlink:href=\"", link, "\" xlink:show=\"replace\">");
    }

  void stopstring() {
    if(link != "") print(f, "</a>");
    }

  string font = "Times";
  
  void text(int x, int y, int size, const string& str, bool frame, color_t col, int align) {

    double dfc = (x - current_display->xcenter) * (x - current_display->xcenter) + 
      (y - current_display->ycenter) * (y - current_display->ycenter);
    dfc /= current_display->radius;
    dfc /= current_display->radius;
    // 0 = center, 1 = edge
    dfc = 1 - dfc;
    
    col = 0xFF + (col << 8);

    bool uselatex = font == "latex";  

    if(!invisible(col)) {
      startstring();
      string str2 = "";
      for(int i=0; i<(int) str.size(); i++)
        if(str[i] == '&')
          str2 += "&amp;";
        else if(str[i] == '<')
          str2 += "&lt;";
        else if(str[i] == '>')
          str2 += "&gt;";
        else if(uselatex && str[i] == '#')
          str2 += "\\#";
        else str2 += str[i];
      if(uselatex) str2 = string("\\myfont{")+coord(size)+"}{" + str2 + "}";  
      
      print(f, "<text x='", coord(x), "' y='", coord(y+size*.4), "' text-anchor='", align == 8 ? "middle" :
        align < 8 ? "start" :
        "end", "' ");
      if(!uselatex)
        print(f, "font-family='", font, "' font-size='", coord(size), "' ");
      print(f, 
        stylestr(col, frame ? 0x0000000FF : 0, (1<<get_sightrange())*dfc/40), 
        ">", str2, "</text>");
      stopstring();
      println(f);
      }
    }
  
  void polygon(int *polyx, int *polyy, int polyi, color_t col, color_t outline, double linewidth) {
  
    if(invisible(col) && invisible(outline)) return;
    if(polyi < 2) return;

    startstring();
    for(int i=0; i<polyi; i++) {
      if(i == 0)
        print(f, "<path d=\"M ");
      else
        print(f, " L ");
      print(f, coord(polyx[i]), " ", coord(polyy[i]));
      }
    
    print(f, "\" ", stylestr(col, outline, (hyperbolic ? current_display->radius : current_display->scrsize) * linewidth/256), "/>");
    stopstring();
    println(f);
    }
  
  void render(const string& fname, const function<void()>& what) {
    dynamicval<bool> v2(in, true);
    dynamicval<bool> v3(vid.usingGL, false);
    
    #if ISWEB
    f.s = "";
    #else
    f.f = fopen(fname.c_str(), "wt");
    #endif

    println(f, "<svg xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" width=\"", coord(vid.xres), "\" height=\"", coord(vid.yres), "\">");
    if(!shot::transparent)
      println(f, "<rect width=\"", coord(vid.xres), "\" height=\"", coord(vid.yres), "\" ", stylestr((backcolor << 8) | 0xFF, 0, 0));
    what();
    println(f, "</svg>");
    
    #if ISWEB
    EM_ASM_({
      var x=window.open();
      x.document.open();
      x.document.write(Pointer_stringify($0));
      x.document.close();
      }, f.s.c_str());
    #else
    fclose(f.f);
    #endif
    }
  
#if CAP_COMMANDLINE
int read_args() {
  using namespace arg;
  if(argis("-svgsize")) {
    shift(); sscanf(argcs(), "%d/%d", &shot::shoty, &svg::divby);
    if(shot::shotformat == -1) shot::shotformat = 0;
    }
  else if(argis("-svgfont")) {
    shift(); svg::font = args();
    // note: use '-svgfont latex' to produce text output as: \myfont{size}{text}
    // (this is helpful with Inkscape's PDF+TeX output feature; define \myfont yourself)
    }
  else if(argis("-svggamma")) {
    shift_arg_formula(shot::gamma);
    }
  else if(argis("-svgshot")) {
    PHASE(3); shift(); start_game();
    printf("saving SVG screenshot to %s\n", argcs());
    svg::render(argcs());
    }
  else return 1;
  return 0;
  }

auto ah = addHook(hooks_args, 0, read_args);
#endif
  }
#endif

#if CAP_PNG
void IMAGESAVE(SDL_Surface *s, const char *fname) {
  SDL_Surface *s2 = SDL_PNGFormatAlpha(s);
  SDL_SavePNG(s2, fname);
  SDL_FreeSurface(s2);
  }
#endif

#if CAP_SHOT
namespace shot {

purehookset hooks_hqshot;

int shotx = 2000, shoty = 2000;
bool make_svg = false;
bool transparent = true;
ld gamma = 1;
int shotformat = -1;
string caption;
ld fade = 1;

void set_shotx() {
  if(shotformat == -1) return;
  shotx = shoty;
  if(shotformat == 1) shotx = shotx * 4/3;
  if(shotformat == 2) shotx = shotx * 16/9;
  if(shotformat == 3) {
    shotx = shotx * 22/16;
    while(shotx & 15) shotx++;
    }
  }

#if CAP_SDL
int shot_aa = 1;
#endif

void default_screenshot_content() {
  #if CAP_RUG
  if(rug::rugged) {
    if(rug::in_crystal()) rug::physics();
    rug::drawRugScene();
    }
  else
  #endif
    drawfullmap();

  if(caption != "")
    displayfr(vid.xres/2, vid.fsize+vid.fsize/4, 3, vid.fsize*2, caption, forecolor, 8);
  callhooks(hooks_hqshot);
  drawStats();    
  }

#if CAP_PNG
void postprocess(string fname, SDL_Surface *sdark, SDL_Surface *sbright) {
  if(gamma == 1 && shot_aa == 1) {
    IMAGESAVE(sdark, fname.c_str());
    return;
    }

  SDL_Surface *sout = SDL_CreateRGBSurface(SDL_SWSURFACE,shotx,shoty,32,0xFF<<16,0xFF<<8,0xFF, (sdark == sbright) ? 0 : (0xFF<<24));
  for(int y=0; y<shoty; y++)
  for(int x=0; x<shotx; x++) {
    int val[2][4];
    for(int a=0; a<2; a++) for(int b=0; b<3; b++) val[a][b] = 0;
    for(int ax=0; ax<shot_aa; ax++) for(int ay=0; ay<shot_aa; ay++)
    for(int b=0; b<2; b++) for(int p=0; p<3; p++)
      val[b][p] += part(qpixel((b?sbright:sdark), x*shot_aa+ax, y*shot_aa+ay), p);
    
    int transparent = 0;
    int maxval = 255 * 3 * shot_aa * shot_aa;
    
    for(int p=0; p<3; p++) transparent += val[1][p] - val[0][p];
    
    color_t& pix = qpixel(sout, x, y);
    pix = 0;
    part(pix, 3) = 255 - (255 * transparent + (maxval/2)) / maxval;
    
    if(transparent < maxval) for(int p=0; p<3; p++) {
      ld v = (val[0][p] * 3. / maxval) / (1 - transparent * 1. / maxval);
      v = pow(v, gamma) * fade;
      v *= 255;
      if(v > 255) v = 255;
      part(pix, p) = v;
      }
    }
  IMAGESAVE(sout, fname.c_str());
  }
#endif

void take(string fname, const function<void()>& what) {

  if(cheater) doOvergenerate();
  
  int multiplier = make_svg ? svg::divby : shot_aa;

  dynamicval<videopar> v(vid, vid);
  dynamicval<bool> v2(inHighQual, true);
  dynamicval<bool> v6(auraNOGL, true);
  vid.smart_range_detail *= multiplier;
  darken = 0;
  
  set_shotx();
  vid.xres = shotx * multiplier;
  vid.yres = shoty * multiplier;
  calcparam();
  
  if(make_svg) {
    #if CAP_SVG
    svg::render(fname, what);
    #endif
    }
  
  else {  
    #if CAP_PNG
    resetbuffer rb;

    renderbuffer glbuf(vid.xres, vid.yres, vid.usingGL);
    glbuf.enable();
    current_display->set_viewport(0);

    dynamicval<color_t> v8(backcolor, transparent ? 0xFF000000 : backcolor);
    #if CAP_RUG
    if(rug::rugged && !rug::renderonce) rug::prepareTexture();
    #endif
    glbuf.clear(backcolor);
    what();
    
    SDL_Surface *sdark = glbuf.render();

    if(transparent) {
      renderbuffer glbuf1(vid.xres, vid.yres, vid.usingGL);
      backcolor = 0xFFFFFFFF;
      #if CAP_RUG
      if(rug::rugged && !rug::renderonce) rug::prepareTexture();
      #endif
      glbuf1.enable();
      glbuf1.clear(backcolor);
      current_display->set_viewport(0);
      what();
      
      postprocess(fname, sdark, glbuf1.render());
      }
    else postprocess(fname, sdark, sdark);
    #endif
    }  
  }

#if CAP_COMMANDLINE
int png_read_args() {
  using namespace arg;
  if(argis("-pngshot")) {
    PHASE(3); shift(); start_game();
    printf("saving PNG screenshot to %s\n", argcs());
    make_svg = false;
    shot::take(argcs());
    }
  else if(argis("-pngsize")) {
    shift(); shoty = argi(); if(shotformat == -1) shotformat = 0;
    }
  else if(argis("-pngformat")) {
    shift(); shotformat = argi();
    }
  else return 1;
  return 0;
  }

auto ah_png = addHook(hooks_args, 0, png_read_args);
#endif

void menu() {
  cmode = sm::SIDE; 
  gamescreen(0);
  if(!CAP_SVG) make_svg = false;
  if(!CAP_PNG) make_svg = true;
  dialog::init(XLAT("screenshots"), iinf[itPalace].color, 150, 100);
  dialog::addSelItem(XLAT("format"), make_svg ? "SVG" : "PNG", 'f');
  dialog::add_action([] { make_svg = !make_svg; });
  dialog::addSelItem(XLAT("pixels (X)"), its(shotx), 'x');
  dialog::add_action([] { shotformat = -1; dialog::editNumber(shotx, 500, 8000, 100, 2000, XLAT("pixels (X)"), ""); });
  dialog::addSelItem(XLAT("pixels (Y)"), its(shoty), 'y');
  dialog::add_action([] { shotformat = -1; dialog::editNumber(shoty, 500, 8000, 100, 2000, XLAT("pixels (Y)"), ""); });
  if(make_svg) {
    using namespace svg;
    dialog::addSelItem(XLAT("precision"), "1/"+its(divby), 'p');
    dialog::add_action([] { divby *= 10; if(divby > 1000000) divby = 1; });
    }
  else {
    dialog::addSelItem(XLAT("supersampling"), its(shot_aa), 's');
    dialog::add_action([] { shot_aa *= 2; if(shot_aa > 16) shot_aa = 1; });
    }
  dialog::addBoolItem(XLAT("transparent"), transparent, 't');
  dialog::add_action([] { transparent = !transparent; });

  dialog::addSelItem(XLAT("gamma"), fts(gamma), 'g');
  dialog::add_action([] { dialog::editNumber(gamma, 0, 2, .1, .5, XLAT("gamma"), "higher value = darker"); });

  dialog::addSelItem(XLAT("brightness"), fts(fade), 'b');
  dialog::add_action([] { dialog::editNumber(fade, 0, 2, .1, 1, XLAT("brightness"), "higher value = lighter"); });

  dialog::addBoolItem(XLAT("show the HUD"), nohud, 'h');
  dialog::add_action([] { nohud = !nohud; });

  dialog::addItem(XLAT("customize colors and aura"), 'c');
  dialog::add_action([] { pushScreen(show_color_dialog); });

  menuitem_sightrange('r');

  dialog::addBreak(100);
  
#if CAP_RUG
  if(make_svg && rug::rugged)
    dialog::addInfo("SVG screenshots do not work in this 3D mode", 0xFF0000);
  else
#endif
#if CAP_TEXTURE
  if(make_svg && texture::config.tstate == texture::tsActive)
    dialog::addInfo("SVG screenshots do not work with textures", 0xFF0000);
  else
#endif
   dialog::addBreak(100);
  
  dialog::addItem(XLAT("take screenshot"), 'z');
  dialog::add_action([] () { 
    #if ISWEB
    shot::take("new window");
    #else
    static string pngfile = "hqshot.png";
    static string svgfile = "svgshot.svg";
    string& file = make_svg ? svgfile : pngfile;
    dialog::openFileDialog(file, XLAT("screenshot"), make_svg ? ".svg" : ".png", [&file] () {
      shot::take(file);
      return true;
      });
    #endif
    });
  dialog::addBack();
  dialog::display();
  }

}
#endif

#if CAP_ANIMATIONS
namespace anims {

enum eMovementAnimation {
  maNone, maTranslation, maRotation, maCircle, maParabolic
  };

eMovementAnimation ma;

ld shift_angle, movement_angle;
ld period = 10000;
int noframes = 30;
ld cycle_length = 2 * M_PI;
ld parabolic_length = 1;
ld skiprope_rotation;

int lastticks, bak_turncount;

ld rug_rotation1, rug_rotation2, ballangle_rotation, env_ocean, env_volcano;
ld rug_angle;

heptspin rotation_center_h;
cellwalker rotation_center_c;
transmatrix rotation_center_View;

color_t circle_display_color = 0x00FF00FF;

ld circle_radius = acosh(2.), circle_spins = 1;

void moved() {
  optimizeview();
  if(cheater || autocheat) {
    if(hyperbolic && memory_saving_mode && cwt.at != centerover.at && !quotient) {
      if(isNeighbor(cwt.at, centerover.at)) {
        cwt.spin = neighborId(centerover.at, cwt.at);
        flipplayer = true;
        }
      animateMovement(cwt.at, centerover.at, LAYER_SMALL, NODIR);
      cwt.at = centerover.at;
      save_memory();
      return;
      }
    setdist(masterless ? centerover.at : viewctr.at->c7, 7 - getDistLimit() - genrange_bonus, NULL);
    }
  playermoved = false;
  }

struct animated_parameter {
  ld *value;
  ld last;
  string formula;
  reaction_t reaction;
  };

vector<animated_parameter> aps;

void deanimate(ld &x) {
  for(int i=0; i<isize(aps); i++) 
    if(aps[i].value == &x)
      aps.erase(aps.begin() + (i--));
  }

void get_parameter_animation(ld &x, string &s) {
  for(auto &ap: aps)
    if(ap.value == &x && ap.last == x)
      s = ap.formula;
  }

void animate_parameter(ld &x, string f, const reaction_t& r) {
  deanimate(x);
  aps.emplace_back(animated_parameter{&x, x, f, r});
  }

int ap_changes;

void apply_animated_parameters() {
  ap_changes = 0;
  for(auto &ap: aps) {
    if(*ap.value != ap.last) continue;
    *ap.value = parseld(ap.formula);
    if(*ap.value != ap.last) {
      if(ap.reaction) ap.reaction();
      ap_changes++;
      ap.last = *ap.value;
      }
    }
  }

bool needs_highqual;

bool joukowsky_anim;

void reflect_view() {
  if(centerover.at) {
    transmatrix T = Id;
    cell *mbase = centerover.at;
    cell *c = centerover.at;
    if(shmup::reflect(c, mbase, T))
      View = inverse(T) * View;
    }
  }


void apply() {
  int t = ticks - lastticks;
  lastticks = ticks;

  switch(ma) {
    case maTranslation:
      if(conformal::on) {
        conformal::phase = (isize(conformal::v) - 1) * ticks * 1. / period;
        conformal::movetophase();        
        }
      else if(centerover.at) {
        reflect_view();
        if((hyperbolic && !quotient && 
          (centerover.at->land != cwt.at->land || memory_saving_mode) && among(centerover.at->land, laHaunted, laIvoryTower, laDungeon, laEndorian) && centerover.at->landparam >= 10
          ) ) {
          if(memory_saving_mode) {
            activateSafety(laIce);
            return;
            }
          else {
            fullcenter(); View = spin(rand() % 1000) * View;
            }
          }
        View = spin(movement_angle * M_PI / 180) * ypush(shift_angle * M_PI / 180) * xpush(cycle_length * t / period) * ypush(-shift_angle * M_PI / 180) * 
          spin(-movement_angle * M_PI / 180) * View;
        moved();
        }
      break;
    case maRotation:
      View = spin(2 * M_PI * t / period) * View;
      break;
    case maParabolic:
      reflect_view();
      View = spin(movement_angle * M_PI / 180) * ypush(shift_angle * M_PI / 180) * binary::parabolic(parabolic_length * t / period) * ypush(-shift_angle * M_PI / 180) * 
        spin(-movement_angle * M_PI / 180) * View;
      moved();
      break;
    case maCircle: {
      if(masterless) centerover = rotation_center_c;
      else viewctr = rotation_center_h;
      ld alpha = circle_spins * 2 * M_PI * ticks / period;
      View = spin(-cos_auto(circle_radius)*alpha) * xpush(circle_radius) * spin(alpha) * rotation_center_View;
      moved();
      break;
      }
    default: 
      break;
    }
  if(env_ocean) {
    turncount += env_ocean * ticks * tidalsize / period;
    calcTidalPhase();
    for(auto& p: gmatrix) if(p.first->land == laOcean) checkTide(p.first);
    turncount -= ticks * tidalsize / period;
    }
  if(env_volcano) {
    bak_turncount = turncount;
    turncount += env_volcano * ticks * 64 / period;
    for(auto& p: gmatrix) if(p.first->land == laVolcano) checkTide(p.first);
    }
  if(rug::rugged) {
    if(rug_rotation1) {
      rug::apply_rotation(rotmatrix(rug_angle * M_PI / 180, 1, 2));
      rug::apply_rotation(rotmatrix(rug_rotation1 * 2 * M_PI * t / period, 0, 2));
      rug::apply_rotation(rotmatrix(-rug_angle * M_PI / 180, 1, 2));
      }
    if(rug_rotation2) {
      rug::apply_rotation(rug::currentrot * rotmatrix(rug_rotation2 * 2 * M_PI * t / period, 0, 1) * inverse(rug::currentrot));
      }
    }
  vid.skiprope += skiprope_rotation * t * 2 * M_PI / period;

  if(ballangle_rotation) {
    if(conformal::model_has_orientation())
      conformal::model_orientation += ballangle_rotation * 360 * t / period;
    else
      vid.ballangle += ballangle_rotation * 360 * t / period;
    }
  if(joukowsky_anim) {
    ld t = ticks / period;
    t = t - floor(t);
    if(pmodel == mdBand) {
      conformal::model_transition = t * 4 - 1;
      }
    else {
      conformal::model_transition = t / 1.1;
      vid.scale = (1 - conformal::model_transition) / 2.;
      }
    }
  apply_animated_parameters();
  if(need_reset_geometry) resetGeometry(), need_reset_geometry = false;
  calcparam();
  }

void rollback() {
  if(env_volcano) {
    turncount = bak_turncount;
    }
  }

#if CAP_FILES && CAP_SHOT
string animfile = "animation-%04d.png";

bool record_animation() {
  lastticks = 0;
  ticks = 0;
  for(int i=0; i<noframes; i++) {
    int newticks = i * period / noframes;
    while(ticks < newticks) shmup::turn(1), ticks++;
    apply();
    conformal::configure();
    if(conformal::on) {
      conformal::phase = isize(conformal::v) * i * 1. / noframes;
      conformal::movetophase();
      }
    
    char buf[1000];
    snprintf(buf, 1000, animfile.c_str(), i);
    shot::take(buf);
    rollback();
    }
  lastticks = ticks = SDL_GetTicks();
  return true;
  }
#endif

void display_animation() {
  if(ma == maCircle && (circle_display_color & 0xFF)) {
    for(int s=0; s<10; s++) {
      if(s == 0) curvepoint(ggmatrix(rotation_center_c.at) * xpush0(circle_radius - .1));
      for(int z=0; z<100; z++) curvepoint(ggmatrix(rotation_center_c.at) * xspinpush0((z+s*100) * 2 * M_PI / 1000., circle_radius));
      queuecurve(circle_display_color, 0, PPR::LINE);
      }
    if(sphere) for(int s=0; s<10; s++) {
      if(s == 0) curvepoint(centralsym * ggmatrix(rotation_center_c.at) * xpush0(circle_radius - .1));
      for(int z=0; z<100; z++) curvepoint(centralsym * ggmatrix(rotation_center_c.at) * xspinpush0((z+s*100) * 2 * M_PI / 1000., circle_radius));
      queuecurve(circle_display_color, 0, PPR::LINE);
      }
    }
  }

void animator(string caption, ld& param, char key) {
  dialog::addBoolItem(caption, param, key);
  if(param) dialog::lastItem().value = fts(param);
  dialog::add_action([&param, caption] () { 
    if(param == 0) {
      param = 1;
      string s = 
        XLAT(
          "The value of 1 means that the period of this animation equals the period set in the animation menu. "
          "Larger values correspond to faster animations.");

      dialog::editNumber(param, 0, 10, 1, 1, caption, s); 
      }
    else param = 0;
    });
  }

ld a, b;

void list_animated_parameters() {
  dialog::addHelp(XLAT(
    "Most parameters can be animated simply by using '..' in their editing dialog. "
    "For example, the value of a parameter set to 0..1 will grow linearly from 0 to 1. "
    "You can also use functions (e.g. cos(0..2*pi)) and refer to other parameters; "
    "parameters 'a' and 'b' exist for this purpose. "
    "See the list below for parameters which are currently animated (or changed)."));
  dialog::addBreak(50);
  for(auto& ap: aps) {
    string what = "?";
    for(auto& p: params) if(&p.second == ap.value) what = p.first;
    dialog::addInfo(what + " = " + ap.formula);
    }
  dialog::addBreak(50);
  dialog::addHelp(parser_help());
  }

ld animation_period;

void show() {
  cmode = sm::SIDE; needs_highqual = false;
  animation_lcm = 1;
  gamescreen(0);
  animation_period = 2 * M_PI * animation_lcm / animation_factor;
  dialog::init(XLAT("animations"), iinf[itPalace].color, 150, 100);
  dialog::addSelItem(XLAT("period"), fts(period)+ " ms", 'p');
  dialog::add_action([] () { dialog::editNumber(period, 0, 10000, 1000, 200, XLAT("period"), 
    XLAT("This is the period of the whole animation, though in some settings the animation can have a different period or be aperiodic. "
      "Changing the value will make the whole animation slower or faster."
    )); });
  if(animation_lcm > 1) {
    dialog::addSelItem(XLAT("game animation period"), fts(animation_period)+ " ms", 'G');
    dialog::add_action([] () {
      dialog::editNumber(animation_period, 0, 10000, 1000, 1000, XLAT("game animation period"), 
        XLAT("Least common multiple of the animation periods of all the game objects on screen, such as rotating items.")
        );
      dialog::reaction = [] () { animation_factor = 2 * M_PI * animation_lcm / animation_period; };
      dialog::extra_options = [] () {
        dialog::addItem("default", 'D');
        dialog::add_action([] () {
          animation_factor = 1;
          popScreen();
          });
        };
      });
    }
  dialog::addBoolItem(XLAT("no movement animation"), ma == maNone, '0');
  dialog::add_action([] () { ma = maNone; });
  dialog::addBoolItem(XLAT("translation"), ma == maTranslation, '1');
  dialog::add_action([] () { ma = maTranslation; });
  dialog::addBoolItem(XLAT("rotation"), ma == maRotation, '2');
  dialog::add_action([] () { ma = maRotation; });
  if(hyperbolic) {
    dialog::addBoolItem(XLAT("parabolic"), ma == maParabolic, '3');
    dialog::add_action([] () { ma = maParabolic; });
    }
  if(among(pmodel, mdJoukowsky, mdJoukowskyInverted)) {
    dialog::addBoolItem(XLAT("joukowsky_anim"), joukowsky_anim, 'j');
    dialog::add_action([] () { joukowsky_anim = !joukowsky_anim; });
    }
  if(among(pmodel, mdJoukowsky, mdJoukowskyInverted)) {
    animator(XLAT("Möbius transformations"), skiprope_rotation, 'S');
    }
  dialog::addBoolItem(XLAT("circle"), ma == maCircle, '4');
  dialog::add_action([] () { ma = maCircle; 
    rotation_center_h = viewctr;
    rotation_center_c = centerover;
    rotation_center_View = View;
    });
  switch(ma) {
    case maCircle: {
      animator(XLAT("circle spins"), circle_spins, 's');
      dialog::addSelItem(XLAT("circle radius"), fts(circle_radius), 'c');
      dialog::add_action([] () { 
        dialog::editNumber(circle_radius, 0, 10, 0.1, acosh(1.), XLAT("circle radius"), ""); 
        dialog::extra_options = [] () {
          if(hyperbolic) {
            // area = 2pi (cosh(r)-1) 
            dialog::addSelItem(XLAT("double spin"), fts(acosh(2.)), 'A');
            dialog::add_action([] () { circle_radius = acosh(2.); });
            dialog::addSelItem(XLAT("triple spin"), fts(acosh(3.)), 'B');
            dialog::add_action([] () { circle_radius = acosh(3.); });
            }
          if(sphere) {
            dialog::addSelItem(XLAT("double spin"), fts(acos(1/2.)), 'A');
            dialog::add_action([] () { circle_radius = acos(1/2.); });
            dialog::addSelItem(XLAT("triple spin"), fts(acos(1/3.)), 'B');
            dialog::add_action([] () { circle_radius = acos(1/3.); });
            }
          };
        });
      dialog::addColorItem(XLAT("draw the circle"), circle_display_color, 'd');
      dialog::add_action([] () {
        dialog::openColorDialog(circle_display_color, NULL);
        });
      dialog::addBreak(100);
      }
    case maTranslation: 
    case maParabolic: {
      if(ma == maTranslation && conformal::on)
        dialog::addBreak(300);
      else if(ma == maTranslation) {
        dialog::addSelItem(XLAT("cycle length"), fts(cycle_length), 'c');
        dialog::add_action([] () { 
          dialog::editNumber(cycle_length, 0, 10, 0.1, 2*M_PI, "shift", ""); 
          dialog::extra_options = [] () {
            dialog::addSelItem(XLAT("full circle"), fts(float(2 * M_PI)), 'A');
            dialog::add_action([] () { cycle_length = 2 * M_PI; });
            dialog::addSelItem(XLAT("Zebra period"), fts(2.898149445355172f), 'B');
            dialog::add_action([] () { cycle_length = 2.898149445355172; });
            dialog::addSelItem(XLAT("Bolza period"), fts(2 * 1.528571f), 'C');
            dialog::add_action([] () { cycle_length = 2 * 1.528571; });
            };
          });
        }
      else {
        dialog::addSelItem(XLAT("cells to go"), fts(parabolic_length), 'c');
        dialog::add_action([] () { 
          dialog::editNumber(parabolic_length, 0, 10, 1, 1, "cells to go", ""); 
          });
        }
      dialog::addSelItem(XLAT("shift"), fts(shift_angle) + "°", 's');
      dialog::add_action([] () { 
        dialog::editNumber(shift_angle, 0, 90, 15, 0, XLAT("shift"), ""); 
        });
      dialog::addSelItem(XLAT("movement angle"), fts(movement_angle) + "°", 'm');
      dialog::add_action([] () { 
        dialog::editNumber(movement_angle, 0, 360, 15, 0, XLAT("movement angle"), ""); 
        });
      break;
      }
    default: {
      dialog::addBreak(300);
      }
    }
  animator(XLATN("Ocean"), env_ocean, 'o');
  animator(XLATN("Volcanic Wasteland"), env_volcano, 'v');

  #if CAP_RUG
  if(rug::rugged) {
    animator(XLAT("screen-relative rotation"), rug_rotation1, 'r');
    if(rug_rotation1) { 
      dialog::addSelItem(XLAT("angle"), fts(rug_angle) + "°", 'a');
      dialog::add_action([] () { 
        dialog::editNumber(rug_angle, 0, 360, 15, 0, "Rug angle", ""); 
        });
      }
    else dialog::addBreak(100);
    animator(XLAT("model-relative rotation"), rug_rotation2, 'r');
    animator(XLAT("automatic move speed"), rug::ruggo, 'M');
    dialog::add_action([] () { 
      dialog::editNumber(rug::ruggo, 0, 10, 1, 1, XLAT("automatic move speed"), XLAT("Move automatically without pressing any keys."));
      if(among(rug::gwhere, gSphere, gElliptic)) 
        dialog::extra_options = [] () {
          dialog::addItem(XLAT("synchronize"), 'S');
          dialog::add_action([] () { rug::ruggo = 2 * M_PI * 1000 / period; popScreen(); });
          };
      });
    }
  #endif
  if(conformal::model_has_orientation())
    animator(XLAT("model rotation"), ballangle_rotation, 'R');
  else if(among(pmodel, mdHyperboloid, mdHemisphere, mdBall))
    animator(XLAT("3D rotation"), ballangle_rotation, '3');
  
  dialog::addSelItem(XLAT("animate parameters"), fts(a), 'a');
  dialog::add_action([] () {
    dialog::editNumber(a, -100, 100, 1, 0, XLAT("animate parameters"), "");
    dialog::extra_options = list_animated_parameters;
    });

  dialog::addSelItem(XLAT("animate parameters"), fts(b), 'b');
  dialog::add_action([] () {
    dialog::editNumber(b, -100, 100, 1, 0, XLAT("animate parameters"), "");
    dialog::extra_options = list_animated_parameters;
    });

  dialog::addBoolItem(XLAT("history mode"), (conformal::on || conformal::includeHistory), 'h');
  dialog::add_action([] () { pushScreen(conformal::history_menu); });

  #if CAP_FILES && CAP_SHOT
  dialog::addItem(XLAT("shot settings"), 's');
  dialog::add_action([] () { pushScreen(shot::menu); });

  if(needs_highqual) 
    dialog::addInfo(XLAT("some parameters will only change in recorded animation"));
  else
    dialog::addBreak(100);
  dialog::addSelItem(XLAT("frames to record"), its(noframes), 'n');
  dialog::add_action([] () { dialog::editNumber(noframes, 0, 300, 30, 5, XLAT("frames to record"), ""); });
  dialog::addSelItem(XLAT("record to a file"), animfile, 'R');
  dialog::add_action([] () { 
    dialog::openFileDialog(animfile, XLAT("record to a file"), shot::make_svg ? ".svg" : ".png", record_animation);
    });
  #endif
  dialog::addBack();
  dialog::display();
  }

#if CAP_COMMANDLINE
int readArgs() {
  using namespace arg;
           
  if(0) ;
  else if(argis("-animmenu")) {
    PHASE(3); showstartmenu = false; pushScreen(show);
    }
  else if(argis("-animperiod")) {
    PHASE(2); shift_arg_formula(period);
    }
  else if(argis("-animrecord")) {
    PHASE(3); shift(); noframes = argi();
    shift(); animfile = args(); record_animation();
    }
  else if(argis("-animcircle")) {
    PHASE(3); start_game();
    ma = maCircle; 
    rotation_center_h = viewctr;
    rotation_center_c = cwt.at;
    rotation_center_View = View;
    shift_arg_formula(circle_spins);
    shift_arg_formula(circle_radius);
    shift(); circle_display_color = arghex();
    }
  else if(argis("-animmove")) {
    ma = maTranslation; 
    shift_arg_formula(cycle_length);
    shift_arg_formula(shift_angle);
    shift_arg_formula(movement_angle);
    }
  else if(argis("-animpar")) {
    ma = maParabolic; 
    shift_arg_formula(parabolic_length);
    shift_arg_formula(shift_angle);
    shift_arg_formula(movement_angle);
    }
  else if(argis("-animrot")) {
    ma = maRotation;
    }
  else if(argis("-animrug")) {
    shift_arg_formula(rug_rotation1);
    shift_arg_formula(rug_angle);
    shift_arg_formula(rug_rotation2);
    }
  else if(argis("-animenv")) {
    shift_arg_formula(env_ocean);
    shift_arg_formula(env_volcano);
    }
  else if(argis("-animball")) {
    shift_arg_formula(ballangle_rotation);
    }
  else if(argis("-animj")) {
    shift(); joukowsky_anim = true;
    }
  else if(argis("-animrec")) {
    PHASE(3);
    shift(); noframes = argi();
    shift(); animfile = args();
    record_animation();
    }

  else return 1;
  return 0;
  }
#endif

auto animhook = addHook(hooks_frame, 100, display_animation)
  #if CAP_COMMANDLINE
  + addHook(hooks_args, 100, readArgs)
  #endif
  ;

bool any_animation() {
  if(conformal::on) return true;
  if(ma) return true;
  if(ballangle_rotation || rug_rotation1 || rug_rotation2) return true;
  if(ap_changes) return true;
  return false;
  }

bool any_on() {
  return any_animation() || conformal::includeHistory;
  }

bool center_music() {
  return among(ma, maParabolic, maTranslation);
  }

}
#endif
}
