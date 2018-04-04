#pragma once
#include "sstream"
#include "vector"
#include "algorithm"
using namespace std;

static string int2string(int n)
{
	stringstream ss;
	ss << n;
	return ss.str();
}

static int string2int(string s)
{
	stringstream ss(s);
	int n;
	ss >> n;
	return n;
}

static string double2string(double d)
{
	stringstream ss;
	ss << d;
	return ss.str();
}

static double string2double(string s)
{
	stringstream ss(s);
	double d;
	ss >> d;
	return d;
}


static void split(const std::string& s, const std::string delim, std::vector< std::string >* ret)
{
	size_t last = 0;
	size_t index = s.find_first_of(delim, last);
	while (index != std::string::npos)
	{
		ret->push_back(s.substr(last, index - last));
		last = index + 1;
		index = s.find_first_of(delim, last);
	}
	if (index - last > 0)
	{
		ret->push_back(s.substr(last, index - last));
	}
}

static size_t levenshtein_distance(const string source, const string target)
{
	//step 1

	size_t  n = source.length();
	size_t m = target.length();
	if (m == 0) return n;
	if (n == 0) return m;
	//Construct a matrix
	typedef vector< vector<int> >  Tmatrix;
	Tmatrix matrix(n + 1);
	for (int i = 0; i <= n; i++)  matrix[i].resize(m + 1);

	//step 2 Initialize

	for (int i = 1; i <= n; i++) matrix[i][0] = i;
	for (int i = 1; i <= m; i++) matrix[0][i] = i;

	//step 3
	for (int i = 1; i <= n; i++)
	{
		const char si = source[i - 1];
		//step 4
		for (int j = 1; j <= m; j++)
		{

			const char dj = target[j - 1];
			//step 5
			int cost;
			if (si == dj){
				cost = 0;
			}
			else{
				cost = 1;
			}
			//step 6
			const int above = matrix[i - 1][j] + 1;
			const int left = matrix[i][j - 1] + 1;
			const int diag = matrix[i - 1][j - 1] + cost;
			matrix[i][j] = std::min(above, min(left, diag));

		}
	}//step7
	return matrix[n][m];
}

