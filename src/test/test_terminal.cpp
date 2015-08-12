#include "iostream"
#include "string"
#include "stdio.h"
#include "sstream"

using namespace std;

string
getTerminalData (string cmd)
{
  FILE* pipe = popen (cmd.c_str (), "r");
  if (!pipe)
    return "ERROR";
  char buffer[128];
  std::string result = "";
  while (!feof (pipe))
  {
    if (fgets (buffer, 128, pipe) != NULL)
      result += buffer;
  }
  pclose (pipe);
  return result;
}

int
main (int argc,
      char** argv)
{

  //string cmd = "rostopic echo /recognized_object_array";
  string cmd = argv[1];

  string result = getTerminalData (cmd);

  istringstream iss (result);
  string line;
  while (std::getline (iss, line))
  {
    cout << "---> " << line << std::endl;
  }

  return 1;
}
