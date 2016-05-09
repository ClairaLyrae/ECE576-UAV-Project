#ifndef CONFIG_H_
#define CONFIG_H_

#include <fstream>
#include <string>
#include <map>
#include <stdlib.h>

using namespace std;

class Config {
private:
	ifstream optionsfile;
	map<string, string> options;
	string str_opt;
	string str_val;
	string str;
public:

	bool load(string fname) {
		char c;
		string temp;
		optionsfile.open(fname.c_str());
		if(!optionsfile.good())
			return false;
		while(!optionsfile.eof()) {
			getline(optionsfile, str);
			c = str.at(0);
			if(c == '#' || c == '\n' || c == '\r' || c == ' ')
				continue;
			stringstream ss(str);
			ss >> str_opt >> temp >> str_val;
			options[str_opt] = str_val;
			// cout << "Loaded config: " << str_opt << " = " << str_val << endl;
		}
		optionsfile.close();
		return true;
	}

	double getDouble(string s) {
		return atof(options[s].c_str());
	}

	string getString(string s) {
		return options[s];
	}

	bool getBool(string s) {
		string t = options[s];
	    return !t.empty () && (strcasecmp (t.c_str (), "true") == 0 || atoi(t.c_str ()) != 0);
	}

	int getInt(string s) {
		return atoi(options[s].c_str());
	}

	void getDouble(string s, double* arr, unsigned len) {
		string temp;
		stringstream ss(options[s]);
		for(unsigned i = 0; i < len && !ss.eof(); i++) {
			getline(ss, temp, ',');
			arr[i] = atof(temp.c_str());
		}
	}

	vector<double> getDoubleVector(string s) {
		string temp;
		vector<double> vals;
		stringstream ss(options[s]);
		while(!ss.eof()) {
			getline(ss, temp, ',');
			vals.push_back(atof(temp.c_str()));
		}
		return vals;
	}
};



#endif
