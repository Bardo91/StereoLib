//
//
//
//
//

#include "LogManager.h"

#include <string>
#include <time.h>


//---------------------------------------------------------------------------------------------------------------------
#ifdef _WIN32
#include <Windows.h>
inline void do_mkdir(std::string _filename) {
	CreateDirectory(_filename.c_str(), NULL);
}
#elif __linux__
#include <sys/stat.h>
#include <sys/types.h>
inline void do_mkdir(std::string _filename) {
	mkdir(_filename.c_str(), 0700);
}
#endif

using namespace std;

// Static data initialization.
LogManager *LogManager::mInstance = nullptr;

//---------------------------------------------------------------------------------------------------------------------
void LogManager::init() {
	if (mInstance == nullptr) {
		mInstance = new LogManager();
	}
}

//---------------------------------------------------------------------------------------------------------------------
LogManager *LogManager::get() {
	return mInstance;
}

//---------------------------------------------------------------------------------------------------------------------
void LogManager::end() {
	for (pair<string, ofstream*> entry : mInstance->mFileMap) {
		entry.second->close();
	}

	delete mInstance;
}

//---------------------------------------------------------------------------------------------------------------------
std::ofstream &LogManager::operator[](const std::string & _file) {
	if (mFileMap.find(mFolderBase+_file) == mFileMap.end()) {
		mFileMap[mFolderBase+_file] = new ofstream(mFolderBase+_file);
	}

	return *mFileMap[mFolderBase+_file];
}

//---------------------------------------------------------------------------------------------------------------------
LogManager::LogManager(){
	mFolderBase = "log_" + to_string(time(NULL)) + "/";
	do_mkdir(mFolderBase.c_str());
}
