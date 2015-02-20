/*
 * InfoFilesVector.h
 *
 *  Created on: Nov 18, 2010
 *      Author: martijn
 */

#ifndef INFOFILESVECTOR_H_
#define INFOFILESVECTOR_H_
//
// ---------------------------------------------------------------------------------------------------------------------
//
#include <vector>
/**
 * The InformationFiles class is used to specify the default and the local information file.
 */
class InformationFiles
{
public:
    /// Constructor
    InformationFiles(std::string defaultInfoFile, std::string localInfoFile)
      : defaultFile(defaultInfoFile), localFile(localInfoFile) {}

    /// Constructor
    InformationFiles()
      : defaultFile(""), localFile("") {}

    /// Destructor
    ~InformationFiles() {};

public:
    std::string defaultFile; ///< The default information filename
    std::string localFile; ///< The local information filename

public:
    bool operator==(const InformationFiles& infoFiles);
    bool hasDuplicateFiles() {return(defaultFile == localFile);}
};

typedef std::vector<InformationFiles> InfoFilesVector; ///< The vector of InformationFiles

#endif // INFOFILESVECTOR_H_
