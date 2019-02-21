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
		// @doc string ת wchar_t* ����ȥɾ���ڴ�
		static wchar_t* StringToWChar(const std::string&);
		static std::string UTF8ToGB(const char*, WCHAR*, LPSTR);
		static std::string StringToUTF8(const std::string&);

		// ���ָ���ļ�·���µ������ļ�
		/*
		* params const char* ·��
		* params std::vector<const char*>& �����ļ� 
		* params const char** �ļ���׺��(��ʱֻ֧��һ�ֻ�ȫ��)
		* parmas bool �Ƿ�ֻҪ�ļ���
		*/
		static void GetFiles(const char*, std::vector<const char*>&, const char* = "\\*.*", bool = true);
#endif // PLATFORM_WIN

	};
}