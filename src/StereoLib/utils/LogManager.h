//
//
//
//
//


#ifndef STEREOLIBS_UTILS_LOGMANAGER_H_
#define STEREOLIBS_UTILS_LOGMANAGER_H_

#include <fstream>
#include <unordered_map>

class LogManager {
public:
	static void init();
	static LogManager *get();
	static void end();


	std::ofstream &operator[](const std::string &_file);

private:
	LogManager();

private:
	std::unordered_map<std::string, std::ofstream*> mFileMap;

	std::string mFolderBase;

	static LogManager *mInstance;
};



#endif // !STEREOLIBS_UTILS_LOGMANAGER_H_
