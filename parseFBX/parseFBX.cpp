//// GenFBXBinary.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
////
//
#include <iostream>
#include <string>
#include <locale>
#include <Windows.h>
#include <fcntl.h>
#include <io.h>
#include <fbxsdk.h>
#include "Common.h"
#include "DLLInterface.h"
#include "DataGenerate.h"


#ifdef UNICODE
// Unicode版本的wmain函数 - 支持特殊字符和宽字符路径
int wmain(int argc, wchar_t* argv[])
#else
// 默认情况下也提供wmain函数，这样用户可以选择使用
int wmain(int argc, wchar_t* argv[])
#endif
{
    // 保存原始编码页
    UINT originalInputCodePage = GetConsoleCP();
    UINT originalOutputCodePage = GetConsoleOutputCP();
    
    // 设置控制台为UTF-8编码
    SetConsoleCP(CP_UTF8);
    SetConsoleOutputCP(CP_UTF8);
    
    // 设置控制台模式支持虚拟终端处理
    HANDLE hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD dwMode = 0;
    GetConsoleMode(hStdOut, &dwMode);
    SetConsoleMode(hStdOut, dwMode | ENABLE_PROCESSED_OUTPUT | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
    
    // 设置C++ locale为UTF-8，使std::cout能正确处理UTF-8字符
    std::locale::global(std::locale(""));
    std::cout.imbue(std::locale());
    std::wcout.imbue(std::locale());
    
    // 注意：不设置_O_U16TEXT模式，保持默认的文本模式
    // 这样std::cout和wprintf都能正常工作

    if (argc != 3 && argc != 4) {
        wprintf(L"Usage: %s <FBX_file_path> <output_directory_path> [parameters]\n", argv[0]);
        wprintf(L"Supports paths with special characters and wide characters\n");
        
        // 演示std::cout也能工作
        std::cout << "std::cout also works with UTF-8 encoding" << std::endl;
        return -1;
    }

    wprintf(L"FBX file path: %s\n", argv[1]);
    wprintf(L"Output directory: %s\n", argv[2]);
    if (argc == 4) {
        wprintf(L"Parameters: %s\n", argv[3]);
    }
    
    // 演示std::cout可以与wprintf混用
    std::cout << "Processing FBX file..." << std::endl;
    fflush(stdout);

    // 转换宽字符参数为UTF-8
    std::string fbxPath = WideToUTF8(argv[1]);
    std::string outPath = WideToUTF8(argv[2]);
    std::string parameter = "";
    
    if (argc == 4) {
        parameter = WideToUTF8(argv[3]);
    }

    ParseFBXW(argv[1], argv[2], argc == 4 ? argv[3] : nullptr);
    
    SetConsoleCP(originalInputCodePage);
    SetConsoleOutputCP(originalOutputCodePage);
    return 0;
}

#ifndef UNICODE
// ANSI版本的main函数
int main(int argc, char** argv)
{
    // 保存原始编码页
    UINT originalInputCodePage = GetConsoleCP();
    UINT originalOutputCodePage = GetConsoleOutputCP();
    
    // 设置控制台为UTF-8编码
    SetConsoleCP(CP_UTF8);
    SetConsoleOutputCP(CP_UTF8);
    
    // 设置控制台模式支持虚拟终端处理
    HANDLE hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD dwMode = 0;
    GetConsoleMode(hStdOut, &dwMode);
    SetConsoleMode(hStdOut, dwMode | ENABLE_PROCESSED_OUTPUT | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
    
    // 设置C++ locale为UTF-8，使std::cout能正确处理UTF-8字符
    std::locale::global(std::locale(""));
    std::cout.imbue(std::locale());

    if (argc != 3 && argc != 4) {
        printf("使用方法: %s <FBX文件路径> <输出目录路径> [参数]\n", argv[0]);
        printf("支持包含特殊字符和宽字符的路径\n");
        
        // 演示std::cout也能工作
        std::cout << "std::cout can also output UTF-8 characters" << std::endl;
        return -1;
    }
    
    char* str = argv[1];
    char* outpath = argv[2];
    char* paramater = nullptr;
    if(argc == 4)
        paramater = argv[3];
    
    printf("FBX文件路径: %s\n", str);
    printf("输出目录: %s\n", outpath);
    if (paramater) {
        printf("参数: %s\n", paramater);
    }
    
    // 演示std::cout可以与printf混用
    std::cout << "Processing FBX file..." << std::endl;
    fflush(stdout);
    
    ParseFBX(str, outpath, paramater);
   
    SetConsoleCP(originalInputCodePage);
    SetConsoleOutputCP(originalOutputCodePage);
    return 0;
}
#endif
