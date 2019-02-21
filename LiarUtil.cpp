#include "LiarUtil.h"

namespace Liar
{
	std::string Liar::LiarUtil::GetSplitLast(const std::string& name, const char* split)
	{
		std::string::size_type pos = name.find_last_of(split);
		if (pos != std::string::npos) return name.substr(pos + 1);
		else return name;
	}

	std::string Liar::LiarUtil::GetSplitHead(const std::string& name, const char* split)
	{
		std::string::size_type pos = name.find_last_of(split);
		if (pos != std::string::npos) return name.substr(0, pos);
		else return name;
	}

	void LiarUtil::GetSplit(const std::string& name, std::string& head, std::string& last, const char* split)
	{
		std::string::size_type pos = name.find(split);
		if (pos != std::string::npos)
		{
			last = name.substr(pos + 1);
			head = name.substr(0, pos);
		}
	}

	bool Liar::LiarUtil::IncludeChinese(const char* str)
	{
		char c;
		while (1)
		{
			c = *str++;
			if (c == 0) break;  //如果到字符串尾则说明该字符串没有中文字符
			if (c & 0x80)        //如果字符高位为1且下一字符高位也是1则有中文字符
				if (*str & 0x80) return true;
		}
		return false;
	}

	void Liar::LiarUtil::SplitString(const std::string& s, std::vector<std::string>& v, const char* c)
	{
		std::string::size_type pos1, pos2;
		pos2 = s.find(c);
		pos1 = 0;
		while (std::string::npos != pos2)
		{
			v.push_back(s.substr(pos1, pos2 - pos1));

			pos1 = pos2 + strlen(c);
			pos2 = s.find(c, pos1);
		}
		if (pos1 != s.length())	v.push_back(s.substr(pos1));
	}

#ifdef PLATFORM_WIN
	std::string Liar::LiarUtil::WCharToString(const wchar_t* wchar)
	{
		DWORD dwNum = WideCharToMultiByte(CP_OEMCP, NULL, wchar, -1, NULL, 0, NULL, FALSE);
		char *psText = new char[dwNum];
		WideCharToMultiByte(CP_OEMCP, NULL, wchar, -1, psText, dwNum, NULL, FALSE);
		std::string out = psText;
		delete[] psText;
		return out;
	}

	// @doc string 转 wchar_t* 外面去删除内存
	wchar_t* Liar::LiarUtil::StringToWChar(const std::string& code)
	{
		const char* pCStrKey = code.c_str();
		//第一次调用返回转换后的字符串长度，用于确认为wchar_t*开辟多大的内存空间
		int pSize = MultiByteToWideChar(CP_OEMCP, 0, pCStrKey, strlen(pCStrKey) + 1, NULL, 0);
		wchar_t *pWCStrKey = new wchar_t[pSize];
		//第二次调用将单字节字符串转换成双字节字符串
		MultiByteToWideChar(CP_OEMCP, 0, pCStrKey, strlen(pCStrKey) + 1, pWCStrKey, pSize);
		return pWCStrKey;
	}

	std::string Liar::LiarUtil::UTF8ToGB(const char* str, WCHAR *strSrc, LPSTR szRes)
	{
		std::string result;

		//获得临时变量的大小
		int i = MultiByteToWideChar(CP_UTF8, 0, str, -1, NULL, 0);
		memset(strSrc, 0, i + 1);
		MultiByteToWideChar(CP_UTF8, 0, str, -1, strSrc, i);

		//获得临时变量的大小
		i = WideCharToMultiByte(CP_ACP, 0, strSrc, -1, NULL, 0, NULL, NULL);
		memset(szRes, 0, i + 1);
		WideCharToMultiByte(CP_ACP, 0, strSrc, -1, szRes, i, NULL, NULL);

		result = szRes;
		return result;
	}

	std::string Liar::LiarUtil::StringToUTF8(const std::string & str)
	{
		int nwLen = ::MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, NULL, 0);

		wchar_t * pwBuf = new wchar_t[nwLen + 1];//一定要加1，不然会出现尾巴
		ZeroMemory(pwBuf, nwLen * 2 + 2);

		::MultiByteToWideChar(CP_ACP, 0, str.c_str(), str.length(), pwBuf, nwLen);

		int nLen = ::WideCharToMultiByte(CP_UTF8, 0, pwBuf, -1, NULL, NULL, NULL, NULL);

		char * pBuf = new char[nLen + 1];
		ZeroMemory(pBuf, nLen + 1);

		::WideCharToMultiByte(CP_UTF8, 0, pwBuf, nwLen, pBuf, nLen, NULL, NULL);

		std::string retStr(pBuf);

		delete[]pwBuf;
		delete[]pBuf;

		pwBuf = NULL;
		pBuf = NULL;

		return retStr;
	}

	void Liar::LiarUtil::GetFiles(const char* lpPath, std::vector<const char*>& files, const char* extName, bool justName)
	{
		char szFind[MAX_PATH];
		WIN32_FIND_DATA FindFileData;

		strcpy_s(szFind, lpPath);
		if (!extName) strcat_s(szFind, "\\*.*");
		else strcat_s(szFind, extName);

		HANDLE hFind = ::FindFirstFile(szFind, &FindFileData);
		if (INVALID_HANDLE_VALUE == hFind)    return;

		while (true)
		{
			if (FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
			{
				if (FindFileData.cFileName[0] != '.')
				{
					char szFile[MAX_PATH];

					strcpy_s(szFile, lpPath);
					strcat_s(szFile, "\\");
					strcat_s(szFile, (char*)(FindFileData.cFileName));
					GetFiles(szFile, files);
				}
			}
			else
			{
				std::string fileName = (char*)(FindFileData.cFileName);
				if (justName)
				{
					files.push_back(fileName.c_str());
				}
				else
				{
					std::string path(fileName);
					path += "\\";
					path += fileName;
					files.push_back(path.c_str());
				}
			}
			if (!FindNextFile(hFind, &FindFileData))    break;
		}
		FindClose(hFind);
	}
#endif // PLATFORM_WIN

}
