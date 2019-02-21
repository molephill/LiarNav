#pragma once

#include <string>
#include <vector>

#define PLATFORM_WIN 1;

#ifdef PLATFORM_WIN
#include <windows.h>
#include <WindowsX.h>
#endif // FPLATFORM_WIN


namespace Liar
{
	class LiarUtil
	{
	public:
		static std::string GetSplitLast(const std::string&, const char* = "\\");
		static std::string GetSplitHead(const std::string&, const char* = ".");
		static void GetSplit(const std::string&, std::string&, std::string&, const char* = ":");
		static bool IncludeChinese(const char*);
		static void SplitString(const std::string&, std::vector<std::string>&, const char* = ",");

#ifdef PLATFORM_WIN
		static std::string WCharToString(const wchar_t*);
		// @doc string 转 wchar_t* 外面去删除内存
		static wchar_t* StringToWChar(const std::string&);
		static std::string UTF8ToGB(const char*, WCHAR*, LPSTR);
		static std::string StringToUTF8(const std::string&);

		// 获得指定文件路径下的所有文件
		/*
		* params const char* 路径
		* params std::vector<const char*>& 所有文件 
		* params const char** 文件后缀名(暂时只支持一种或全部)
		* parmas bool 是否只要文件名
		*/
		static void GetFiles(const char*, std::vector<const char*>&, const char* = "\\*.*", bool = true);
#endif // PLATFORM_WIN

	};
}