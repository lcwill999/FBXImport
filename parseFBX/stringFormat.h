#pragma once
#include <iostream>
#include <string.h>
#include <stdarg.h>
#include <string>
#include <vector>
#include <sstream>
#include <Windows.h>

inline void ReportError(std::string ss)
{
	std::cout << " [Error] " << ss << std::endl;
}

inline void ReportWarning(std::string ss)
{
	std::cout << " [Warning] " << ss << std::endl;
}

// 宽字符版本的报告函数
inline void ReportErrorW(const std::wstring& ws)
{
	std::wcout << L" [Error] " << ws << std::endl;
}

inline void ReportWarningW(const std::wstring& ws)
{
	std::wcout << L" [Warning] " << ws << std::endl;
}

inline std::string VFormat(const char* format, va_list ap)
{
	va_list zp;
	va_copy(zp, ap);
	char buffer[1024 * 10];
	vsnprintf(buffer, 1024 * 10, format, zp);
	va_end(zp);
	return buffer;
}

inline std::string Format(const char* format, ...)
{
	va_list va;
	va_start(va, format);
	std::string formatted = VFormat(format, va);
	va_end(va);
	return formatted;
}

// 宽字符版本的格式化函数
inline std::wstring VFormatW(const wchar_t* format, va_list ap)
{
	va_list zp;
	va_copy(zp, ap);
	wchar_t buffer[1024 * 10];
	vswprintf(buffer, 1024 * 10, format, zp);
	va_end(zp);
	return buffer;
}

inline std::wstring FormatW(const wchar_t* format, ...)
{
	va_list va;
	va_start(va, format);
	std::wstring formatted = VFormatW(format, va);
	va_end(va);
	return formatted;
}

// 字符串转换工具函数
namespace StringUtils {
	
	// 宽字符转UTF-8
	inline std::string WideToUTF8(const std::wstring& wideStr) {
		if (wideStr.empty()) return std::string();
		
		int sizeNeeded = WideCharToMultiByte(CP_UTF8, 0, wideStr.c_str(), -1, NULL, 0, NULL, NULL);
		if (sizeNeeded <= 0) return std::string();
		
		std::string utf8Str(sizeNeeded - 1, 0);
		WideCharToMultiByte(CP_UTF8, 0, wideStr.c_str(), -1, &utf8Str[0], sizeNeeded, NULL, NULL);
		return utf8Str;
	}

	// UTF-8转宽字符
	inline std::wstring UTF8ToWide(const std::string& utf8Str) {
		if (utf8Str.empty()) return std::wstring();
		
		int sizeNeeded = MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, NULL, 0);
		if (sizeNeeded <= 0) return std::wstring();
		
		std::wstring wideStr(sizeNeeded - 1, 0);
		MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, &wideStr[0], sizeNeeded);
		return wideStr;
	}

	// ANSI转宽字符
	inline std::wstring AnsiToWide(const std::string& ansiStr) {
		if (ansiStr.empty()) return std::wstring();
		
		int sizeNeeded = MultiByteToWideChar(CP_ACP, 0, ansiStr.c_str(), -1, NULL, 0);
		if (sizeNeeded <= 0) return std::wstring();
		
		std::wstring wideStr(sizeNeeded - 1, 0);
		MultiByteToWideChar(CP_ACP, 0, ansiStr.c_str(), -1, &wideStr[0], sizeNeeded);
		return wideStr;
	}

	// 宽字符转ANSI
	inline std::string WideToAnsi(const std::wstring& wideStr) {
		if (wideStr.empty()) return std::string();
		
		int sizeNeeded = WideCharToMultiByte(CP_ACP, 0, wideStr.c_str(), -1, NULL, 0, NULL, NULL);
		if (sizeNeeded <= 0) return std::string();
		
		std::string ansiStr(sizeNeeded - 1, 0);
		WideCharToMultiByte(CP_ACP, 0, wideStr.c_str(), -1, &ansiStr[0], sizeNeeded, NULL, NULL);
		return ansiStr;
	}

	// 路径规范化函数
	inline std::wstring NormalizePath(const std::wstring& path) {
		std::wstring normalized = path;
		
		// 替换正斜杠为反斜杠
		for (size_t i = 0; i < normalized.length(); i++) {
			if (normalized[i] == L'/') {
				normalized[i] = L'\\';
			}
		}
		
		// 移除末尾的反斜杠
		if (!normalized.empty() && normalized.back() == L'\\') {
			normalized.pop_back();
		}
		
		return normalized;
	}

	// 安全路径检查
	inline bool IsValidPath(const std::wstring& path) {
		if (path.empty()) return false;
		
		// 检查路径中的非法字符
		const std::wstring invalidChars = L"<>:\"|?*";
		for (wchar_t c : invalidChars) {
			if (path.find(c) != std::wstring::npos) {
				return false;
			}
		}
		
		// 检查路径长度
		if (path.length() > MAX_PATH) {
			return false;
		}
		
		return true;
	}

	// 获取文件扩展名
	inline std::wstring GetFileExtension(const std::wstring& filePath) {
		size_t pos = filePath.find_last_of(L'.');
		if (pos != std::wstring::npos && pos < filePath.length() - 1) {
			return filePath.substr(pos + 1);
		}
		return L"";
	}

	// 获取文件名（不包含扩展名）
	inline std::wstring GetFileNameWithoutExtension(const std::wstring& filePath) {
		size_t slashPos = filePath.find_last_of(L'\\');
		if (slashPos == std::wstring::npos) {
			slashPos = filePath.find_last_of(L'/');
		}
		
		size_t start = (slashPos == std::wstring::npos) ? 0 : slashPos + 1;
		size_t dotPos = filePath.find_last_of(L'.');
		size_t end = (dotPos == std::wstring::npos || dotPos <= start) ? filePath.length() : dotPos;
		
		return filePath.substr(start, end - start);
	}

	// 获取目录路径
	inline std::wstring GetDirectoryPath(const std::wstring& filePath) {
		size_t pos = filePath.find_last_of(L'\\');
		if (pos == std::wstring::npos) {
			pos = filePath.find_last_of(L'/');
		}
		
		if (pos != std::wstring::npos) {
			return filePath.substr(0, pos);
		}
		return L"";
	}
}