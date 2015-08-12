#include "iostream"
#include "fstream"
#include "string"
#include "vector"

using namespace std;

int
main (int argc,
      char** argv)
{

  fstream f1, f2;
  f1.open (argv[1]);
  if (!f1.is_open ())
  {
    cout << "Can't open f1 file" << endl;
    return -1;
  }
  f2.open (argv[2]);
  if (!f2.is_open ())
  {
    cout << "Can't open f2 file" << endl;
    return -1;
  }

  string header1[11], header2[11];
  for (int i = 0; i < 11; i++)
  {
    getline (f1, header1[i]);
    getline (f2, header2[i]);
  }

  //cout << header1[10] << endl;
  vector<string> allData;
  vector<string> objData;
  string temp;
  while (!f1.eof ())
  {
    getline (f1, temp);
    if (temp != "")
      allData.push_back (temp);
  }

  while (!f2.eof ())
  {
    getline (f2, temp);
    if (temp != "")
      objData.push_back (temp);
  }
  f1.close ();
  f2.close ();

  cout << allData.size () << endl;
  cout << objData.size () << endl;
  cout << allData[allData.size () - 1] << endl;

  int arr[307200] = { 0 };

  // look for index
  for (int i = 0; i < objData.size (); i++)
  {
    for (int j = 0; j < 307200; j++)
    {
      if (objData[i] == allData[j])
      {
        if (arr[j] == 1)
        {
          cout << "Duplicate points happens!";
          return -1;
        }
        arr[j] = 1;
      }
    }
  }

  ofstream fout (argv[3]);
  if (!fout)
  {
    cout << "Can't open output file" << endl;
    return -1;
  }
  for (int i = 0; i < 11; i++)
    fout << header1[i] << endl;

  for (int i = 0; i < 307200; i++)
  {
    if (arr[i] == 0)  // need to replace with NaN point
      fout << "nan nan nan 1234567" << endl;
    else
      fout << allData[i] << endl;  //keep it!

  }

  fout.close ();
  cout << "all done!";

  return 1;
}
