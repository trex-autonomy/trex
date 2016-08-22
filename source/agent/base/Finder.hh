/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2009. Willow Garage.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TREX Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


#include <fstream>
#include "Utilities.hh"
#include "XMLUtils.hh"
#include <string.h>

namespace TREX {

  class Finder {
  public:
    Finder(const char* name): m_name(name){}

    ~Finder(){}

    int main(int argc, char **argv) {

      bool show_path = false;
      bool is_valid = true;
      bool dir_enabled = false;
      bool print_help = false;
      char * start_dir = 0;

      if(argc < 2 || argc > 6){
	printHelp();
	return 2;
      }

      for(int i = 1; i < argc; i++){
	// If it is arg 1, it should be the file. Invalid if it is a marker
	if(i == 1 && strncmp(argv[i], "--", 2) == 0){
	  is_valid = false;
	}

	if(strcmp(argv[i], "--help") == 0){
	  print_help = true;
	}
	else if(strcmp(argv[i], "--dir") == 0){
	  if(dir_enabled){
	    printHelp();
	    return 2;
	  }
	  dir_enabled = true;
	}
	else if(strcmp(argv[i], "--path") == 0){
	  show_path = true;
	}
	else if(dir_enabled){
	  if(start_dir == 0)
	    start_dir = argv[i];
	  else{
	    printHelp();
	    return 2;
	  }
	}
      }

      if(dir_enabled && start_dir == 0){
	printHelp();
	return 2;
      }

      // Set up the start directory if appropriate.
      char mypath[1024], wd[1024];
      getcwd(wd, 1024);
      if(start_dir != 0){
	chdir(start_dir);
      }

      setenv("TREX_START_DIR", getcwd(mypath, 1024), 1);
      chdir(wd);

      // Load configuration files to configure search path
      std::ifstream f1(TREX::findFile("NDDL.cfg").c_str());
      std::ifstream f2(TREX::findFile("temp_nddl_gen.cfg").c_str());
      TiXmlElement* iroot = NULL;
      if (f1.good()) {
	iroot = EUROPA::initXml(TREX::findFile("NDDL.cfg").c_str());
      } else if (f2.good()) {
	iroot = EUROPA::initXml(TREX::findFile("temp_nddl_gen.cfg").c_str());
      } else {
	printf("Could not find 'NDDL.cfg' or 'temp_nddl_gen.cfg', thus, not searching a path.\n");
      }
      if (iroot) {
	for (TiXmlElement * ichild = iroot->FirstChildElement();
	     ichild != NULL;
	     ichild = ichild->NextSiblingElement()) {
	  if (std::string(ichild->Value()) == "include") {
	    std::string path = std::string(ichild->Attribute("path"));
	    for (unsigned int i = 0; i < path.size(); i++) {
	      if (path[i] == ';') {
		path[i] = ':';
	      }
	    }
	    setenv("TREX_PATH", path.c_str(), 1);
	  }
	}
      }

      if(is_valid){
	std::string f = TREX::findFile(argv[1], true);
	std::ifstream fi(f.c_str());
	if (fi.good()) {
	  execute(f);
	} else {
	  printf("File '%s' not found.\n", f.c_str());
	  printSearchPath();
	  return 1;
	}
      }

      if(show_path)
	printSearchPath();

      if(print_help)
	printHelp();

      return 0;
    }

  protected:
    virtual void execute(const std::string& file) = 0;

  private:
    void printHelp(){
      printf("%s is a utility to locate a file in the trex search path.\n", m_name);
      printf("\n");
      printf("Usage:  %s file [--dir start_dir]  [--help] [--path]\n", m_name);
      printf("  file   The file name to search for.\n");
      printf("  --dir  Specify a start_dir in which to start the search from. If omitted, we will assume the search starts in the current directory.\n");
      printf("  --help Produces this menu.\n");
      printf("  --path Displays the search path.\n\n");
    }

    void printSearchPath(){
      printf("Path: %s.\n\n", getenv("TREX_PATH"));
      printf("Start Dir: %s.\n", getenv("TREX_START_DIR"));
    }

    const char* m_name;
  };
}
