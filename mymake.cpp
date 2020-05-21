// HyperRogue: alternative build system
// This reads the file 'hyper.cpp' and compiles the cpp files it includes into separate object files, and then links them.
// Tested in Linux, should work in other systems with some changes.

// Options:
//   -O2 -- optimize
//   -O3 -- optimize
//   -D... -- change compilation flags
//   [file.cpp] -- add a module to the build (e.g. ./mymake rogueviz)

#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <sys/stat.h>
#include <unistd.h>

using namespace std;

string opts;

int batch_size = 6; // how many modules to compile in parallel

string default_standard = " -std=c++11";
string standard = default_standard;

string preprocessor;
string compiler;
string linker;
string libs;

void set_linux() {
  preprocessor = "g++ -E";
  compiler = "g++ -Wall -Wextra -Wno-maybe-uninitialized -Wno-unused-parameter -Wno-implicit-fallthrough -rdynamic -fdiagnostics-color=always -c";
  linker = "g++ -rdynamic -o hyper";
  opts = "-DFHS -DLINUX -I/usr/include/SDL";
  libs = " savepng.o -lSDL -lSDL_ttf -lSDL_mixer -lSDL_gfx -lGLEW -lGL -lpng -rdynamic -lpthread -lz";
  }

void set_mac() {
  preprocessor = "g++ -E";
  compiler = "g++ -march=native -W -Wall -Wextra -pedantic -Wno-unused-parameter -Wno-implicit-fallthrough -c";
  linker = "g++ -o hyper";
  opts = "-DMAC -I/usr/local/include";
  libs = " savepng.o -L/usr/local/lib -framework AppKit -framework OpenGL -lSDL -lSDLMain -lSDL_gfx -lSDL_mixer -lSDL_ttf -lpng -lpthread -lz";
  }

void set_win() {
  preprocessor = "g++ -E";
  compiler = "runbat bwin-g.bat -c";
  linker = "runbat bwin-linker.bat";
  opts = "-DFHS -DLINUX -I/usr/include/SDL";
  libs = "";

  standard = "";
  }

vector<string> modules;

time_t get_file_time(const string s) {
  struct stat attr;
  if(stat(s.c_str(), &attr)) return 0;
  return attr.st_mtime;
  }

int optimized = 0;

string obj_dir = "mymake_files";
string setdir = "../";

int system(string cmdline) {
  return system(cmdline.c_str());
  }

bool file_exists(string fname) {
  return access(fname.c_str(), F_OK) != -1;
  }
  
int main(int argc, char **argv) {
#if defined(MAC)
  set_mac();
#elif defined(WINDOWS)
  set_win();
#else
  set_linux();
#endif
  for(string fname: {"Makefile.loc", "Makefile.simple", "Makefile"})
    if(file_exists(fname)) {
      system("make -f " + fname + " language-data.cpp autohdr.h savepng.o");
      break;
      }
  for(int i=1; i<argc; i++) {
    string s = argv[i];
    if(s.substr(0, 2) == "-D") {
      opts += " " + s;
      obj_dir += "/";
      setdir += "../";
      for(char c: s) 
        if(!isalnum(c)) obj_dir += "_"; 
        else obj_dir += c;      
      }
    else if(s == "-win") {
      set_win();
      obj_dir += "/win";
      setdir += "../";
      }
    else if(s == "-mac") {
      set_mac();
      obj_dir += "/mac";
      setdir += "../";
      }
    else if(s == "-linux") {
      set_linux();
      obj_dir += "/linux";
      setdir += "../";
      }
    else if(s.substr(0, 2) == "-f") {
      opts += " " + s;
      obj_dir += "/";
      setdir += "../";
      for(char c: s) 
        if(!isalnum(c)) obj_dir += "_"; 
        else obj_dir += c;
      linker += " " + s;
      }
    else if(s == "-O2")
      optimized = 2, compiler += " -O2", obj_dir += "/O2", setdir += "../";
    else if(s == "-O3")
      optimized = 3, compiler += " -O3", obj_dir += "/O3", setdir += "../";
    else if(s.substr(0, 4) == "-std")
      standard = s;
    else if(s.substr(0, 2) == "-l")
      linker += " " + s;
    else if(s.substr(0, 2) == "-j")
      batch_size = stoi(s.substr(2));
    else if(s == "-I") {
      opts += " " + s + " " + argv[i+1];
      i++;
      }
    else if(s.substr(0, 2) == "-I")
      opts += " " + s;
    else if(s == "-rv") {
      
      if(standard == default_standard) {
        standard = "-std=c++17";
        }
      ifstream ifs("rogueviz/rogueviz-all.cpp");
      string s;
      while(getline(ifs, s)) {
        if(s.substr(0, 10) == "#include \"") {
          string t = s.substr(10);
          t = t.substr(0, t.find(".cpp\""));
          modules.push_back("rogueviz/" + t);
          }
        }
      }
    else {
      if(s.size() >= 5 && s.substr(s.size() - 4) == ".cpp")
        s = s.substr(0, s.size() - 4);
      modules.push_back(s);
      }
    }
  if(!optimized)
    compiler += " -g3";
  preprocessor += " " + standard;
  compiler += " " + standard;
  ifstream fs("hyper.cpp");

  system("mkdir -p " + obj_dir);

  ofstream fsm(obj_dir + "/hyper.cpp");
  fsm << "#if REM\n#define INCLUDE(x)\n#endif\n";
  string s;
  while(getline(fs, s)) {
    if(s.substr(0, 8) == "#include") {
      string t;
      bool in = false;
      bool ext = false;
      string iext = "";
      for(char c: s) if(c == '"') in = !in; else if(!in) ; else if(c == '.') ext = !ext; else if(!ext) t += c; else iext += c;
      if(iext == "h") { fsm << "#include \"" + setdir + "hyper.h\"\n"; continue; }
      if(iext != "cpp") printf("unknown extension: %s\n", iext.c_str());
      fsm << "INCLUDE(\"" << t << "\")\n";
      continue;
      }
    fsm << s << "\n";
    }
  fsm.close();
  
  printf("preprocessing...\n");
  if(system(preprocessor + " " + opts + " "+obj_dir+"/hyper.cpp -o "+obj_dir+"/hyper.E")) { printf("preprocessing error\n"); exit(1); }
  
  if(true) {
    ifstream fs2(obj_dir+"/hyper.E");
    while(getline(fs2, s)) {
      if(s.substr(0, 7) == "INCLUDE") {
        s = s.substr(9);
        s = s.substr(0,s.size() - 2);
        modules.push_back(s);
        }
      }
    }

  if(get_file_time(obj_dir + "/hyper.o") < get_file_time("hyper.cpp")) {
    printf("compiling hyper...\n");
    if(system(compiler + " -DREM " + opts + " " + obj_dir + "/hyper.cpp -c -o " + obj_dir + "/hyper.o")) { printf("error\n"); exit(1); }
    }
  
  string allobj = " " + obj_dir + "/hyper.o";

  const int module_amt = modules.size();
  for (int i = 0; i < module_amt; i += batch_size) {
    string batch_cmd = "";
    string batch_str = "";
    for (int j = i; j < i + batch_size; j++) {
      if (j >= module_amt) break;
      string m = modules[j];
      string src = m + ".cpp";
      string m2 = m;
      for(char& c: m2) if(c == '/') c = '_';
      string obj = obj_dir + "/" + m2 + ".o";
      time_t src_time = get_file_time(src);
      if(!src_time) { 
        printf("file not found: %s\n", src.c_str());
        exit(1);
        }
      time_t obj_time = get_file_time(obj);
      if(src_time > obj_time) {
        batch_str += "compiling " + m + " [" + to_string(j+1) + "/" + to_string(module_amt) + "]\n";
        batch_cmd += compiler + " " + opts + " " + src + " -o " + obj + " &";
        }
      else {
        batch_str += "ok: " + m + "\n";
        }
      allobj += " ";
      allobj += obj;
      }
    batch_cmd += "wait";
    printf("%s", batch_str.c_str());
    if (system(batch_cmd)) { printf("compilation error!\n"); exit(1); }
    }
  
  printf("linking...\n");
  system(linker + allobj + libs);
  return 0;
  }
