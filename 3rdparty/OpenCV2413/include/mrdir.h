#ifndef MRDIR_H
#define MRDIR_H
#include "string"
#include "iostream"
#include "vector"
using namespace std;

#ifdef WIN32
#include <tchar.h>
#include "windows.h"

static wchar_t * ANSIToUnicode(const char* str)
{
	int textlen;
	wchar_t * result;
	textlen = MultiByteToWideChar(CP_ACP, 0, str, -1, NULL, 0);
	result = (wchar_t *)malloc((textlen + 1)*sizeof(wchar_t));
	memset(result, 0, (textlen + 1)*sizeof(wchar_t));
	MultiByteToWideChar(CP_ACP, 0, str, -1, (LPWSTR)result, textlen);
	return result;
}

static char * UnicodeToANSI(const wchar_t* str)
{
	char* result;
	int textlen;
	textlen = WideCharToMultiByte(CP_ACP, 0, str, -1, NULL, 0, NULL, NULL);
	result = (char *)malloc((textlen + 1)*sizeof(char));
	memset(result, 0, sizeof(char)* (textlen + 1));
	WideCharToMultiByte(CP_ACP, 0, str, -1, result, textlen, NULL, NULL);
	return result;
}

static wchar_t * UTF8ToUnicode(const char* str)
{
	int textlen;
	wchar_t * result;
	textlen = MultiByteToWideChar(CP_UTF8, 0, str, -1, NULL, 0);
	result = (wchar_t *)malloc((textlen + 1)*sizeof(wchar_t));
	memset(result, 0, (textlen + 1)*sizeof(wchar_t));
	MultiByteToWideChar(CP_UTF8, 0, str, -1, (LPWSTR)result, textlen);
	return result;
}

static char * UnicodeToUTF8(const wchar_t* str)
{
	char* result;
	int textlen;
	textlen = WideCharToMultiByte(CP_UTF8, 0, str, -1, NULL, 0, NULL, NULL);
	result = (char *)malloc((textlen + 1)*sizeof(char));
	memset(result, 0, sizeof(char)* (textlen + 1));
	WideCharToMultiByte(CP_UTF8, 0, str, -1, result, textlen, NULL, NULL);
	return result;
}

static char* ANSIToUTF8(const char* str)
{
	return UnicodeToUTF8(ANSIToUnicode(str));
}

static char* UTF8ToANSI(const char* str)
{
	return UnicodeToANSI(UTF8ToUnicode(str));
}
#if _UNICODE
static std::wstring Ansi2WChar(LPCSTR pszSrc, int nLen)
{
	int nSize = MultiByteToWideChar(CP_ACP, 0, (LPCSTR)pszSrc, nLen, 0, 0);
	if (nSize <= 0) return NULL;

	WCHAR *pwszDst = new WCHAR[nSize + 1];
	if (NULL == pwszDst) return NULL;

	MultiByteToWideChar(CP_ACP, 0, (LPCSTR)pszSrc, nLen, pwszDst, nSize);
	pwszDst[nSize] = 0;

	if (pwszDst[0] == 0xFEFF)                    // skip Oxfeff
	for (int i = 0; i < nSize; i++)
		pwszDst[i] = pwszDst[i + 1];

	wstring wcharString(pwszDst);
	delete pwszDst;

	return wcharString;
}

static std::wstring s2ws(const string& s)
{
	return Ansi2WChar(s.c_str(), s.size());
}
static string WChar2Ansi(LPCWSTR pwszSrc)
{
	int nLen = WideCharToMultiByte(CP_ACP, 0, pwszSrc, -1, NULL, 0, NULL, NULL);

	if (nLen <= 0) return std::string("");

	char* pszDst = new char[nLen];
	if (NULL == pszDst) return std::string("");

	WideCharToMultiByte(CP_ACP, 0, pwszSrc, -1, pszDst, nLen, NULL, NULL);
	pszDst[nLen - 1] = 0;

	std::string strTemp(pszDst);
	delete[] pszDst;

	return strTemp;
}
#endif

//获取当前文件夹下所有子文件夹，保存在vector<string>中
static bool getAllSubdirs(string strDir, vector<string> &subdirs)
{
	WIN32_FIND_DATA FindData;
	HANDLE hError;
	string file2find = strDir + "/*.*";
#if _UNICODE
	hError = FindFirstFile((LPCTSTR)s2ws((LPCSTR)(file2find).c_str()).c_str(), &FindData);
#else
	hError = FindFirstFile((LPCTSTR)file2find.c_str(), &FindData);
#endif
	if (hError == INVALID_HANDLE_VALUE)
	{
		std::cout << "未获得文件夹" << std::endl;
		return 0;
	}
	else
	{
		do
		{
			//过滤.和..;“.”代表本级目录“..”代表父级目录;
			if (lstrcmp(FindData.cFileName, TEXT(".")) == 0 || lstrcmp(FindData.cFileName, TEXT("..")) == 0)
			{
				continue;
			}
			//读取文件属性，如果不是文件夹;
			if (!(FindData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
			{
				continue;
			}
			//如果是文件夹
			if (FindData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
			{
#if _UNICODE
				subdirs.push_back(WChar2Ansi(FindData.cFileName));
#else
				subdirs.push_back(FindData.cFileName);
#endif
			}
			//循环，查找下一个文件;

		} while (::FindNextFile(hError, &FindData));
	}
	//关闭句柄;
	FindClose(hError);
	return 0;
}

//获取目录下所有的文件
static bool getAllFilesinDir(string strDir, vector<string> &files,string ext="*.*")//获取当前文件夹下所有子文件夹，保存在vector<string>中
{
	WIN32_FIND_DATA FindData;
	HANDLE hError;
	string file2find = strDir + "/"+ext;
#if _UNICODE
	hError = FindFirstFile((LPCTSTR)s2ws((LPCSTR)(file2find).c_str()).c_str(), &FindData);
#else
	hError = FindFirstFile((LPCTSTR)file2find.c_str(), &FindData);
#endif
	if (hError == INVALID_HANDLE_VALUE)
	{
		return 0;
	}
	else
	{
		do
		{
			if (lstrcmp(FindData.cFileName, TEXT(".")) == 0 || lstrcmp(FindData.cFileName, TEXT("..")) == 0)
			{
				continue;
			}
			else if (FindData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
			{
				continue;
			}
			else if (!(FindData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
			{
#if _UNICODE
				files.push_back(WChar2Ansi(FindData.cFileName));
#else
				files.push_back(FindData.cFileName);
#endif
			}
		} while (::FindNextFile(hError, &FindData));
	}
	FindClose(hError);
	return 0;
}

#include "io.h"
static bool exist(const char*filepath)
{
	if ((_access(filepath, 0)) != -1)
	{
		return true;
	}
	else
	{
		return false;
	}
}

#else

#include <dirent.h>  
#include <sys/types.h>  
#include <sys/stat.h>  
#include <unistd.h>

static bool getAllSubdirs(string strDir, vector<string> &subdirs)
{
	DIR *dp;                      // 定义子目录流指针  
	struct dirent *entry;         // 定义dirent结构指针保存后续目录  
	struct stat statbuf;          // 定义statbuf结构保存文件属性  
	if((dp = opendir(strDir.c_str())) == NULL) // 打开目录，获取子目录流指针，判断操作是否成功  
	{    
		return 0;  
	}                    // 切换到当前目录 
	while((entry = readdir(dp)) != NULL)  // 获取下一级目录信息，如果未否则循环  
	{  
		lstat(entry->d_name, &statbuf); // 获取下一级成员属性  
		if(S_IFDIR &statbuf.st_mode)    // 判断下一级成员是否是目录  
		{
			if (strcmp(".", entry->d_name) == 0 || strcmp("..", entry->d_name) == 0)  
				continue;  
			subdirs.push_back(entry->d_name); 
		}  
	}                                                    // 回到上级目录  
	closedir(dp);                                                 // 关闭子目录流 
	return 0;
}

static bool getAllFilesinDir(string strDir, vector<string> &files,string ext="*.*")
{
	struct dirent* ent = NULL;  
	DIR *pDir;  
	pDir = opendir(strDir.c_str());  
	if (pDir == NULL) {  
		return false;  
	}
	while (NULL != (ent = readdir(pDir))) {  
		if (ent->d_type == 8) {  
			files.push_back(ent->d_name);
		}
		else {
			if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0) {
				continue;
			}
		}
	}
	closedir(pDir);  
	return true;
}
#endif

#endif
