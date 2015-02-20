#ifndef FILELOGGER_H
#define FILELOGGER_H

#include <wordexp.h>

#include <string>
#include <fstream>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <stdio.h>
#include <time.h>

namespace Loggers
{
    class Logger
    {
    public:
        typedef boost::shared_ptr<Logger> Ptr ;
        virtual void Log(const std::string& logMessage)=0;
        virtual ~Logger() {}
    };

    class EmptyFileLogger : public Logger
    {
    public:
        typedef boost::shared_ptr<EmptyFileLogger> Ptr ;
        void Log(const std::string &logMessage)
        {

        }
    };

    class FileLogger : public Logger
    {
    public:
        typedef boost::shared_ptr<FileLogger> Ptr ;
        FileLogger(const std::string& fileName) : m_fileName(fileName)
        {
            boost::filesystem::path path(fileName);
            boost::filesystem::path dirs = path.branch_path();

            if(!dirs.empty() && !boost::filesystem::exists(dirs))
            {
                boost::filesystem::create_directories(dirs);
            }
        }

        static void appendCwd(const char *path, char *dst) {
            if (path[0]=='/') {
                strcpy(dst, path);
            } else {
                getcwd(dst, PATH_MAX);
                strcat(dst, "/");
                strcat(dst, path);
            }
        }

        static void removeJunk(char *begin, char *end) {
            while(*end!=0) { *begin++ = *end++; }
            *begin = 0;
        }

        static char *manualPathFold(char *path) {
            char *s, *priorSlash;
            while ((s=strstr(path, "/../"))!=NULL) {
                *s = 0;
                if ((priorSlash = strrchr(path, '/'))==NULL) { /* oops */ *s = '/'; break; }
                removeJunk(priorSlash, s+3);
            }
            while ((s=strstr(path, "/./"))!=NULL) { removeJunk(s, s+2); }
            while ((s=strstr(path, "//"))!=NULL) { removeJunk(s, s+1); }
            s = path + (strlen(path)-1);
            if (s!=path && *s=='/') { *s=0; }
            return path;
        }

        std::string abspath(const char *file_name, char *resolved_name)
        {

            char buff[PATH_MAX+1];

            wordexp_t p;

            if (wordexp(file_name, &p, 0)==0) {

                appendCwd(p.we_wordv[0], buff);

                wordfree(&p);

            } else {

                appendCwd(file_name, buff);

            }

            if (realpath(buff, resolved_name)==NULL) { strcpy(resolved_name, manualPathFold(buff)); }

            return resolved_name;

        }

        void Log(const std::string& logMessage)
        {
            time_t rawtime;
            struct tm * timeinfo;

            time ( &rawtime );
            timeinfo = localtime ( &rawtime );

            std::string timeString(asctime (timeinfo));
            boost::algorithm::trim(timeString);

            // TODO: this code should be tested and refactored - I found implementation of the
            // "abspath" on the internet, and copied it as it was ! Never do that :).
            std::string result ;
            abspath(m_fileName.c_str(),const_cast<char*>(result.c_str()));

            m_file.open(result.c_str(),std::ios::out | std::ios::ate | std::ios::app);
            std::ostream os(&m_file);
            os << "[" << timeString << "] " << logMessage << std::endl;
            m_file.close();
        }
    private:
        std::string m_fileName ;
        std::filebuf m_file;
    };
};

#endif // FILELOGGER_H
