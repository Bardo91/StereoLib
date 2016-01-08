//
//
//
//
//

#include "LogManager.h"

#include <string>
#include <time.h>

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
LogManager &LogManager::get() {
	return *mInstance;
}

//---------------------------------------------------------------------------------------------------------------------
void LogManager::end() {
	for (pair<string, ofstream> entry : mInstance->mFileMap) {
		entry.second.close();
	}

	delete mInstance;
}

//---------------------------------------------------------------------------------------------------------------------
std::ofstream &LogManager::operator[](const std::string & _file) {
	if (mFileMap.find(mFolderBase+_file) == mFileMap.end()) {
		mFileMap[mFolderBase+_file] = ofstream(mFolderBase+_file);
	}

	return mFileMap[mFolderBase+_file];
}

//---------------------------------------------------------------------------------------------------------------------
LogManager::LogManager(){
	mFolderBase = "log_" + to_string(time(NULL)) + "/";
}
