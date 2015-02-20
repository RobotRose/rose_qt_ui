#include "DataStore/dataStore.h"

#include <cstdio>
#include <cerrno>
#include <cassert>
#include <algorithm>
#include <string>
#include <limits>
#include <set>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xmlsave.h>

#include "DataStore/SWExceptions.h"

#include "DataStore/dataStoreItemConverter.h"

// File extensions used to write the XML file in a fail-safe way.
static const std::string TEMP_FILE_EXTENSION = ".tmp";
static const std::string LOCK_FILE_EXTENSION = ".lck";

// XML tags
static const std::string TAG_INFORMATION = "information";
static const std::string TAG_MODULE = "module";
static const std::string TAG_ITEM = "item";
static const std::string TAG_ITEMSET = "itemset";
static const std::string TAG_ID = "id";
static const std::string TAG_ITEMTYPE = "type";
static const std::string TAG_ITEMVALUE = "value";
//
//-------------------------------------------------------------------------------------------------
//
bool CmpInfoFilesAtLeastOneEqual(InformationFiles first, InformationFiles second)
{
  return ((first.defaultFile == second.defaultFile) ||
          (first.defaultFile == second.localFile) ||
          (first.localFile == second.defaultFile) ||
          (first.localFile == second.localFile));
}
//
//-------------------------------------------------------------------------------------------------
//
bool InformationFiles::operator==(const InformationFiles& m_infoFiles)
{
  return ((defaultFile == m_infoFiles.defaultFile) && (localFile == m_infoFiles.localFile));
}
//
//-------------------------------------------------------------------------------------------------
//
DataStore::DataStore(std::string componentTag) :
                m_activeTransaction(0) ,
                m_initialized(false) ,
                m_tag_component(componentTag) 
{
  if (pthread_mutex_init(&m_dataLock, NULL) != 0) 
  {
    DS_SW_ERROR("ERROR: Could not initialize mutex");
    return;
  }
  
  m_defaultInfoMap = new InfoMap();
  m_localInfoMap = new InfoMap();
  DS_LOG_GEN_TRACE("New DataStore() instance");
}
//
//-------------------------------------------------------------------------------------------------
//
DataStore::~DataStore() 
{
  lock();
  // Release the in-memory representation of the XML file.

  while (!m_defaultXmlDocs.empty()) 
  {
    xmlFreeDoc(m_defaultXmlDocs.back());
    m_defaultXmlDocs.pop_back();
  }
  while (!m_localXmlDocs.empty()) 
  {
    xmlFreeDoc(m_localXmlDocs.back());
    m_localXmlDocs.pop_back();
  }
  delete m_defaultInfoMap;
  delete m_localInfoMap;
  unlock();
}
//
//-------------------------------------------------------------------------------------------------
//
// initialize for single file -- For backwards compatibility
int DataStore::initialize(std::string localInfo)
{
  InfoFilesVector informationFiles;
  informationFiles.push_back(InformationFiles("", localInfo));
  return (initialize(informationFiles));
}
//
//-------------------------------------------------------------------------------------------------
//
// initialize for default and local file -- For backwards compatibility
int DataStore::initialize(std::string defaultInfo, std::string localInfo) 
{
  InfoFilesVector informationFiles;
  informationFiles.push_back(InformationFiles(defaultInfo, localInfo));
  return (initialize(informationFiles));
}
//
//-------------------------------------------------------------------------------------------------
//
// initialize for multiple default and local files
int DataStore::initialize(InfoFilesVector& informationFiles)
{
  if (informationFiles.empty())
  {
    DS_LOG_GEN_TRACE("File list empty");
    return 1;
  }

  // Find duplicates in informationFiles
  bool duplicatesFound(false);
  for (InfoFilesVector::iterator iter = informationFiles.begin(); 
       ((iter != informationFiles.end()) && !duplicatesFound);  
       ++iter)
  {
    duplicatesFound = iter->hasDuplicateFiles();
  }
  if (!duplicatesFound)
  {
    InfoFilesVector dummyFilesVector(informationFiles.size());
    duplicatesFound = unique_copy(
        informationFiles.begin(),
        informationFiles.end(),
        dummyFilesVector.begin(),
        CmpInfoFilesAtLeastOneEqual) != dummyFilesVector.end();
  }
  if (duplicatesFound)
  {
    DS_SW_ERROR("File list contains duplicates");
    return 1;
  }
  
  DS_LOG_GEN_TRACE("Initialize DataStore...");
  m_infoFiles = informationFiles;

  // Clear the info maps.
  m_defaultInfoMap->clear();
  m_localInfoMap->clear();

  int categoryNr = 0;
  for (InfoFilesVector::const_iterator infoFilestIter = m_infoFiles.begin(); 
       infoFilestIter != m_infoFiles.end();  
       ++infoFilestIter)
  {
    DS_LOG_GEN_TRACE("Initialize maps for category %d", categoryNr);

    xmlDoc* defaultXmlDoc = NULL;
    xmlDoc* localXmlDoc = NULL;

    if (infoFilestIter->defaultFile != "")
    {
      // Recover from the situation when the last write action to the XML file was interrupted.
      // Load default information
      if (recoverFile(infoFilestIter->defaultFile) != 0)
      {
        return 1;
      }
 
      if (loadFromFile(infoFilestIter->defaultFile, categoryNr, defaultXmlDoc, m_defaultInfoMap) != 0)
      {
        return 1;
      }
    }
  
    // Load local information
    if (infoFilestIter->localFile != "") 
    {
      if (recoverFile(infoFilestIter->localFile) != 0)
      {
        return 1;
      }

      if (infoFilestIter->defaultFile != "")
      {
        // Only when we have a default file, the local file might not be there and we'll have to create one.
        FILE *temp = fopen(infoFilestIter->localFile.c_str(), "r");
        if (temp == NULL) 
        {
          DS_LOG_GEN_TRACE("Warning: creating new local information file %s", infoFilestIter->localFile.c_str());
          temp = fopen(infoFilestIter->localFile.c_str(), "w");
          if (temp == NULL)
          {
            DS_SW_ERROR("ERROR: Could not create local information file %s", infoFilestIter->localFile.c_str());
            return 1;
          }
          fprintf(temp, "<?xml version=\"1.0\"?>\n<%s>\n</%s>\n", TAG_INFORMATION.c_str(), TAG_INFORMATION.c_str());
        } 
        fclose(temp);
      }
        
      if (loadFromFile(infoFilestIter->localFile, categoryNr, localXmlDoc, m_localInfoMap) != 0)
      {
        return 1;
      } 

      if (infoFilestIter->defaultFile != "")
      {
        // Check consistency with the default file.
        checkConsistency();
      }
    }
    else
    {
      DS_SW_ERROR("ERROR: Local file specified with empty string.");
      return 1;
    }

    m_defaultXmlDocs.push_back(defaultXmlDoc);
    m_localXmlDocs.push_back(localXmlDoc);
    ++categoryNr;
  }    

  m_initialized = true;
  return 0;  
}  
//
//-------------------------------------------------------------------------------------------------
//
// Check if all items in local also exist in default and remove it from local if not.
void DataStore::checkConsistency() 
{
  // Use the maps to find incorrect items
  InfoMap::iterator itLocal = m_localInfoMap->begin();
  InfoMap::iterator itDefault;
  while (itLocal != m_localInfoMap->end()) 
  {
    itDefault = m_defaultInfoMap->find(itLocal->first);
    if (itDefault == m_defaultInfoMap->end()) 
    { // item is not found in default map -> remove it from local map
      // remove from xml tree
      DS_LOG_GEN_TRACE("WARNING: Item %s in Local information file is not found in default information file. It is removed.",
              (itLocal->first).c_str());
      xmlNode *node = itLocal->second.node;
      xmlUnlinkNode(node);
      xmlFreeNode(node);
      // remove from map
      m_localInfoMap->erase(itLocal);
      itLocal = m_localInfoMap->begin(); // start over again
    }
    else 
    {
      itLocal++;
    }
  }
  return;
}
//
//-------------------------------------------------------------------------------------------------
//
int DataStore::recoverFile(std::string filename) 
{
  int retVal = 0;
  
  // Is there a lock file?
  if (!lockFileExists(filename))
  {
    // No lock file found. So were not finished with the writing procedure of the XML file. The original .xml file should still be there.
    // Is there a (partial) temporary file?
    if (tempFileExists(filename))
    {
      // Temporary file found. Try to remove the temporary file.
      retVal = removeTempFile(filename);
      if (retVal == 0)
      {
        sync(); // Flush file system buffers to mark this point on disk.
      }
    }
  }
  else
  {
    // Lock file found. So were interrupted in the middle of the write procedure.
    // We are however sure that the temporary file contains correct data.
    
    DS_LOG_GEN_TRACE("WARNING: Recovering file: %s", filename.c_str());
    
    // Try to finished the interrupted write to file procedure.
    // Is the temporary file still there?
    // Is there a temporary file?
    if (tempFileExists(filename))
    {
      // Temporary file found.  Is the original XML file still there?
      if (originalFileExists(filename))
      {
        // Try to remove the original file.
        retVal = removeOriginalFile(filename);
        if (retVal == 0)
        {
          sync(); // Flush file system buffers to mark this point on disk.
        }
      }
      if (retVal == 0)
      {
        // Try to rename the temp file to the original file.
        retVal = renameTempFile(filename);
        if (retVal == 0)
        {
          sync(); // Flush file system buffers to mark this point on disk.
        }
      }
    }
    else
    {
      ; // Assume the temporary file is already renamed to the original file name. Just remove the lock file.
    }
    if (retVal == 0)
    {
      // Try to remove the lock file.
      retVal = removeLockFile(filename);
      if (retVal == 0)
      {
        sync(); // Flush file system buffers to mark this point on disk.
      }
    }
  }
  if (retVal != 0)
  {
    DS_SW_ERROR("ERROR: Recovery failed for file: %s", filename.c_str());
    retVal = 1;
  }
  return retVal;
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::lockFileExists(std::string filename) const
{
  bool bExists(false);
  std::string lockXMLfile = filename + LOCK_FILE_EXTENSION;
  
  // Is the lock file still there?
  FILE* fLockFile = fopen(lockXMLfile.c_str(), "r");
  if (fLockFile != NULL)
  {
    // Lock file found.
    fclose(fLockFile);
    
    bExists = true;
  }
  return bExists;
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::tempFileExists(std::string filename) const
{
  bool bExists(false);
  std::string tempXMLfile = filename + TEMP_FILE_EXTENSION;
  
  // Is the temporary file still there?
  FILE* fTempFile = fopen(tempXMLfile.c_str(), "r");
  if (fTempFile != NULL)
  {
    // Temporary file found.
    fclose(fTempFile);
    
    bExists = true;
  }
  return bExists;
}
//
//-------------------------------------------------------------------------------------------------
//
int DataStore::loadFromFile(std::string filename, int category, xmlDoc*& xmlDoc, InfoMap* map)
{
  int retVal(0);

  DS_LOG_GEN_TRACE("Reading input file: %s -- m_tag_component=%s", filename.c_str(), m_tag_component.c_str());
  
  xmlDoc = xmlReadFile(filename.c_str(), NULL, 0);
  
  if(xmlDoc == NULL)
  {
    DS_SW_ERROR("ERROR: Failed to read from input file: %s!", filename.c_str());
    retVal = -1;
  }
  else
  {
    xmlNode* root_element = xmlDocGetRootElement(xmlDoc);
    for (xmlNode* cur_node = root_element; cur_node != NULL; cur_node = cur_node->next) 
    {
      if (cur_node->type == XML_ELEMENT_NODE)
      {
        if ((char*)cur_node->name == TAG_INFORMATION) // == overloaded for std::string comparison
        {
          retVal = readInformation(category, cur_node, map);
        }
      }
    } // for
    xmlCleanupParser();
    // Keep the defaultXmlDoc in memory!
  }
  if (retVal != 0)
  {
    DS_LOG_GEN_TRACE("ERROR: Loading failed for file: %s", filename.c_str());
  }
  return retVal;
}
//
//-------------------------------------------------------------------------------------------------
//

int DataStore::writeToFile(std::string filename, xmlDoc *xmldoc)
{
  DS_LOG_GEN_TRACE("Writing to file: %s", filename.c_str());
  
  std::string tempXMLfile = filename + TEMP_FILE_EXTENSION;
  
  // Write XML tree first to a temporary file.
  xmlSaveCtxtPtr ctxt = xmlSaveToFilename(tempXMLfile.c_str(), NULL, 0);
  if (xmlSaveDoc(ctxt, xmldoc) == -1) 
  {
    DS_SW_ERROR("ERROR: Failed to xmlSaveDoc");
    return -1;
  }
  if (xmlSaveClose( ctxt ) == -1)
  {
    DS_SW_ERROR("ERROR: Failed to write temp file: %s", tempXMLfile.c_str());
    return -1;
  }

  sync(); // Flush file system buffers to mark this point on disk.
    
  // Temporary XML file successfully written.
  // Try to write a lock file to make recovery possible (e.g when power is switched off)
  if (writeLockFile(filename) != 0)
  {
    DS_SW_ERROR("Could not write lock file");
    return -1;
  }

  sync(); // Flush file system buffers to mark this point on disk.

  // Try to remove the original file. This is no problem because we are sure that the temporary file and lock file has been written to disk.
  if (removeOriginalFile(filename) != 0)
  {
    DS_LOG_GEN_TRACE("Could not remove original file");
    return -1;
  } 

  sync(); // Flush file system buffers to mark this point on disk.
    
    // Try to rename the temp file to the original file.
  if (renameTempFile(filename) != 0) 
  {
    DS_SW_ERROR("Could not rename temp file");
    return -1;
  } 
  
  sync(); // Flush file system buffers to mark this point on disk.
    
    // Try to remove the lock file.
  if (removeLockFile(filename) != 0) 
  {
    DS_SW_ERROR("Could not remove lock file");
    return -1;
  }
  
  sync(); // Flush file system buffers to mark this point on disk.
  return 0;
}
//
//-------------------------------------------------------------------------------------------------
//
int DataStore::removeOriginalFile(std::string filename)
{
  int retVal(0);
  // Try to remove the original XML file.
  if ( remove(filename.c_str()) != 0 )
  {
    DS_LOG_GEN_TRACE("ERROR: Failed to remove XML file: %s", filename.c_str());
    retVal = -1;
  }
  return retVal;
}     
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::originalFileExists(std::string filename) const
{
  bool bExists(false);
  FILE* fFile = fopen(filename.c_str(), "r");
  if (fFile != NULL)
  {
    // Original file found.
    fclose(fFile);
    
    bExists = true;
  }
  return bExists;
}
//
//-------------------------------------------------------------------------------------------------
//
int DataStore::writeLockFile(std::string filename)
{
  int retVal(0);
  std::string lockXMLfile = filename + LOCK_FILE_EXTENSION;

  // Write a lock file to make recovery possible (e.g when power is switched off)
  FILE* fLockFile = fopen(lockXMLfile.c_str(), "w");
  if (fLockFile == NULL)
  {
    DS_LOG_GEN_TRACE("ERROR: Failed to write lock file: %s", lockXMLfile.c_str());
    retVal = -1;
  }
  else
  {
    fprintf(fLockFile, "LOCK\n");
    fclose(fLockFile);
  }
  return retVal;
}
//
//-------------------------------------------------------------------------------------------------
//
int DataStore::removeLockFile(std::string filename)
{
  int retVal(0);
  std::string lockXMLfile = filename + LOCK_FILE_EXTENSION;
  
  // Try to remove the lock file.
  if ( remove(lockXMLfile.c_str()) != 0 )
  {
    DS_LOG_GEN_TRACE("ERROR: Failed to remove lock file: %s", lockXMLfile.c_str());
    retVal = -1;
  }
  return retVal;
}     
//
//-------------------------------------------------------------------------------------------------
//
int DataStore::removeTempFile(std::string filename)
{
  int retVal(0);
  std::string tempXMLfile = filename + TEMP_FILE_EXTENSION;

  // Try remove the temporary file.
  if ( remove(tempXMLfile.c_str()) != 0 )
  {
    DS_LOG_GEN_TRACE("ERROR: Failed to remove temp file: %s", tempXMLfile.c_str());
    retVal = -1;
  }
  return retVal;
}     
//
//-------------------------------------------------------------------------------------------------
//
int DataStore::renameTempFile(std::string filename)
{
  int retVal(0);
  std::string tempXMLfile = filename + TEMP_FILE_EXTENSION;
  
  // Try to rename the temporary file.
  int res = rename(tempXMLfile.c_str(), filename.c_str());
  if (res != 0)
  {
    DS_LOG_GEN_TRACE("ERROR: Failed to rename file: %s to %s", tempXMLfile.c_str(), filename.c_str());
    DS_LOG_GEN_TRACE("error code: %d", res);
    DS_LOG_GEN_TRACE("error message: %d", errno);
    retVal = -1;
  }
  return retVal;
}
//
//-------------------------------------------------------------------------------------------------
//
// XML helper methods.
int DataStore::readInformation(int category, xmlNode* node, InfoMap *map)
{
  for (xmlNode* cur_node = node->children; cur_node != NULL; cur_node = cur_node->next) 
  {
    if (cur_node->type == XML_ELEMENT_NODE)
    {
      if ((char*)cur_node->name == m_tag_component) // == overloaded for std::string comparison
      {
        std::string componentID = "";
        if (readComponent(category, cur_node, componentID, map) != 0)
        {
          return -1;
        }
      }
      else 
      {
        DS_SW_ERROR("ERROR: Illegal information file: tag %s not within a %s", (char*)cur_node->name, m_tag_component.c_str());
        return -1;
      }
    }
  } // for
  return 0;
}
//
//-------------------------------------------------------------------------------------------------
//
int DataStore::readComponent(int category, xmlNode* node, std::string componentID, InfoMap *map)
{
  std::string newComponentID;
  std::string itemID = "";
  if (componentID == "") 
  { // first component level
    newComponentID = getID(node);
  }
  else
  { // not the first component level, concatenate
    newComponentID = componentID + SEPARATOR + getID(node);
  }
  
  for (xmlNode* cur_node = node->children; cur_node != NULL; cur_node = cur_node->next) 
  {
    if (cur_node->type == XML_ELEMENT_NODE)
    {
      if ((char*)cur_node->name == m_tag_component) // == overloaded for std::string comparison
      {
        // recursively compose componentID
        if (readComponent(category, cur_node, newComponentID, map) != 0)
        {
          return -1;
        }
      } 
      else if ((char*)cur_node->name == TAG_ITEMSET) // == overloaded for std::string comparison
      {
        if (readItemset(category, cur_node, newComponentID, itemID, map) != 0)
        {
          return -1;
        }
      }
      else if ((char*)cur_node->name == TAG_ITEM) // == overloaded for std::string comparison
      {
        readInfo(category, cur_node, newComponentID, itemID, map);
      }
      else 
      {
        DS_SW_ERROR("ERROR: Illegal information file: a %s can only contain %ss and items, not a %s", m_tag_component.c_str(), m_tag_component.c_str(), (char*)cur_node->name);
        return -1;
      }
    }
  } // for
  return 0;
}
//
//-------------------------------------------------------------------------------------------------
//
int DataStore::readItemset(int category, xmlNode* node, const std::string componentID, std::string itemID, InfoMap *map)
{
  std::string newItemID;
  if (itemID == "") 
  { // first component level
    newItemID = getID(node);
  }
  else
  { // not the first component level, concatenate
    newItemID = itemID + SEPARATOR + getID(node);
  }
  
  for (xmlNode* cur_node = node->children; cur_node != NULL; cur_node = cur_node->next) 
  {
    if (cur_node->type == XML_ELEMENT_NODE)
    {
      if ((char*)cur_node->name == TAG_ITEMSET) // == overloaded for std::string comparison
      {
        // recursively compose componentID
        if (readItemset(category, cur_node, componentID, newItemID, map) != 0)
        {
          return -1;
        }
      } 
      else if ((char*)cur_node->name == TAG_ITEM) // == overloaded for std::string comparison
      {
        readInfo(category, cur_node, componentID, newItemID, map);
      }
      else 
      {
        DS_SW_ERROR("ERROR: Illegal information file: an itemset can only contain itemsets and items, not a %s", (char*)cur_node->name);
        return -1;
      }
    }
  } // for
  return 0;
}
//
//-------------------------------------------------------------------------------------------------
//
void DataStore::readInfo(int category, xmlNode* node, const std::string& componentID, const std::string itemID, InfoMap *map)
{
  std::string infoID;
  if (itemID == "")
  {
    infoID = getID(node);
  }
  else
  {
    infoID = itemID + SEPARATOR + getID(node);
  }
  std::string infoType  = getInfoType(node);
  std::string infoValue = getInfoValue(node);

  // Store information item in lookup table.
  InfoData infoTypeAndValue = {category, node, infoType, infoValue};
  InfoKey infoKey = composeInfoKey(componentID, infoID);

  if (map->count(infoKey) != 0)
  {
    DS_LOG_GEN_TRACE("WARNING: Duplicate information item %s (type %s) existing value %s, new value %s",
      infoKey.c_str(), infoType.c_str(), (*map)[infoKey].infoValue.c_str(), infoValue.c_str());
  }

  map->insert(InfoPair(infoKey, infoTypeAndValue));
  // DS_LOG_GEN_TRACE("Setting %s to %s (%s)", infoKey.c_str(), infoValue.c_str(), infoType.c_str());
}
//
//-------------------------------------------------------------------------------------------------
//
std::string DataStore::getID(xmlNode* node)
{
  xmlChar* value = xmlGetProp(node, (const xmlChar*)TAG_ID.c_str());
  std::string retval("");
  if (value != NULL)
  {
    retval = (char*)value;
    xmlFree(value);
  }
  else
  { // property is not found
    DS_LOG_GEN_TRACE("ERROR: Property '%s' not found on tag %s", TAG_ID.c_str(), node->name);
  }
  return retval;
}
//
//-------------------------------------------------------------------------------------------------
//
std::string DataStore::getInfoType(xmlNode* node)
{
  xmlChar* value = xmlGetProp(node, (const xmlChar*)TAG_ITEMTYPE.c_str());
  std::string retval("");
  if (value != NULL)
  {
    // Add t_ to the type std::string. It is removed in the xml file to increase readability, but cannot be removed from the enum items.
    retval = (char*)value;
    xmlFree(value);
  }
  else
  {
    DS_LOG_GEN_TRACE("ERROR: Property '%s' not found on tag %s", TAG_ITEMTYPE.c_str(), node->name);
  }
  return retval;
}
//
//-------------------------------------------------------------------------------------------------
//
std::string DataStore::getInfoValue(xmlNode* node)
{
  xmlChar* value = xmlGetProp(node, (const xmlChar*)TAG_ITEMVALUE.c_str());
  std::string retval("");
  if (value != NULL)
  {
    retval = (char*)value;
    xmlFree(value);
  }
  else
  {
    DS_LOG_GEN_TRACE("ERROR: Property '%s' not found on tag %s", TAG_ITEMVALUE.c_str(), node->name);
  }
  return retval;
}
//
//-------------------------------------------------------------------------------------------------
//
InfoKey DataStore::composeInfoKey(const std::string& componentID,  const std::string& infoID)
{
  return (componentID + KEY_STRING_SEPARATOR + infoID);
}
//
//-------------------------------------------------------------------------------------------------
//
//////////////////// get or set data items ///////////////////
bool DataStore::getItemValue(std::string componentID, std::string infoID, std::string& value) 
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  InfoData item;
  if (findItem(componentID, infoID, item))
  {
    if (item.infoType != InfoTypeString())
    {
      DS_SW_ERROR("ERROR: Requesting infoItem %s;%s as std::string, which is of type %s", componentID.c_str(), infoID.c_str(), item.infoType.c_str());
      return false;
    }
    value = item.infoValue;
    return true;
  }
  else 
  {
    return false;
  }
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::getItemValue(std::string componentID, std::string infoID, int& value) 
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  InfoData item;
  if (findItem(componentID, infoID, item))
  {
    if (item.infoType != InfoTypeInt())
    {
      DS_SW_ERROR("ERROR: Requesting infoItem %s;%s as integer, which is of type %s", componentID.c_str(), infoID.c_str(), item.infoType.c_str());
      return false;
    }
    value = DataStoreItemConverter::itemStringToInteger(item.infoValue);
    return true;
  }
  else 
  {
    return false;
  }
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::getItemValue(std::string componentID, std::string infoID, bool& value) 
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  InfoData item;
  if (findItem(componentID, infoID, item))
  {
    if (item.infoType != InfoTypeBool())
    {
      DS_SW_ERROR("ERROR: Requesting infoItem %s;%s as boolean, which is of type %s", componentID.c_str(), infoID.c_str(), item.infoType.c_str());
      return false;
    }
    value = DataStoreItemConverter::itemStringToBoolean(item.infoValue);
    return true;
  }
  else 
  {
    return false;
  }
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::getItemValue(std::string componentID, std::string infoID, double& value) 
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  InfoData item;
  if (findItem(componentID, infoID, item))
  {
    if (item.infoType != InfoTypeDouble())
    {
      DS_SW_ERROR("ERROR: Requesting infoItem %s;%s as double, which is of type %s", componentID.c_str(), infoID.c_str(), item.infoType.c_str());
      return false;
    }
    value = DataStoreItemConverter::itemStringToDouble(item.infoValue);
    return true;
  }
  else 
  {
    return false;
  }
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::getItemValue(std::string componentID, std::string infoID, std::list<int>& value)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  InfoData item;
  if (findItem(componentID, infoID, item))
  {
    value.clear();
    value = DataStoreItemConverter::itemStringToListOfInteger(item.infoValue);  
    return true;
  }
  else 
  {
    return false;
  }
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::getItemValue(std::string componentID, std::string infoID, std::list<std::string>& value)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  InfoData item;
  if (findItem(componentID, infoID, item))
  {
    value.clear();
    value = DataStoreItemConverter::itemStringToListOfString(item.infoValue); 

    return true;
  }
  else 
  {
    return false;
  }
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::getItemValue(std::string componentID, std::string infoID, std::list<bool>& value)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  InfoData item;
  if (findItem(componentID, infoID, item))
  {
    value.clear();
    value = DataStoreItemConverter::itemStringToListOfBoolean(item.infoValue);  
    return true;
  }
  else 
  {
    return false;
  }
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::getItemValue(std::string componentID, std::string infoID, std::list<double>& value)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  InfoData item;
  if (findItem(componentID, infoID, item))
  {
    value.clear();
    value = DataStoreItemConverter::itemStringToListOfDouble(item.infoValue); 
    return true;
  }
  else 
  {
    return false;
  }
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::setItemValue(std::string componentID, std::string infoID, const std::string& value, int transactionId)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  std::string newValue = value;
  // no active transaction: write to file, otherwise, don't
  bool write = false;
  if (transactionId == 0)
  {
    lock();
    write = true;
  }
  else
  {
    if (transactionId != m_activeTransaction)
    {
      DS_SW_ERROR("ERROR: Someone pretends to be in a transaction, but did not start one");
      return false;
    }
  }
  bool res = setItemValueWorker(componentID, infoID, newValue, write);
  if (transactionId == 0)
  {
    unlock();
  }
  return res;
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::setItemValue(std::string componentID, std::string infoID, const int value, int transactionId)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  bool write = false;
  if (transactionId == 0)
  {
    lock();
    write = true;
  }
  else
  {
    if (transactionId != m_activeTransaction)
    {
      DS_SW_ERROR("ERROR: Someone pretends to be in a transaction, but did not start one");
      return false;
    }
  }
  std::string newValue = DataStoreItemConverter::itemIntegerToString(value);
  bool res = setItemValueWorker(componentID, infoID, newValue, write);
  if (transactionId == 0)
  {
    unlock();
  }
  return res;
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::setItemValue(std::string componentID, std::string infoID, const double value, int transactionId)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  bool write = false;
  if (transactionId == 0)
  {
    lock();
    write = true;
  }
  else
  {
    if (transactionId != m_activeTransaction)
    {
      DS_SW_ERROR("ERROR: Someone pretends to be in a transaction, but did not start one");
      return false;
    }
  }
  std::string newValue = DataStoreItemConverter::itemDoubleToString(value);
  bool res = setItemValueWorker(componentID, infoID, newValue, write);
  if (transactionId == 0)
  {
    unlock();
  }
  return res;
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::setItemValue(std::string componentID, std::string infoID, const bool value, int transactionId)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  bool write = false;
  if (transactionId == 0)
  {
    lock();
    write = true;
  }
  else
  {
    if (transactionId != m_activeTransaction)
    {
      DS_SW_ERROR("ERROR: Someone pretends to be in a transaction, but did not start one");
      return false;
    }
  }
  std::string newValue = DataStoreItemConverter::itemBooleanToString(value);
  bool res = setItemValueWorker(componentID, infoID, newValue, write);
  if (transactionId == 0)
  {
    unlock();
  }
  return res;
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::setItemValue(std::string componentID, std::string infoID, const std::list<int>& value, int transactionId)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  bool write = false;
  if (transactionId == 0)
  {
    lock();
    write = true;
  }
  else
  {
    if (transactionId != m_activeTransaction)
    {
      DS_SW_ERROR("ERROR: Someone pretends to be in a transaction, but did not start one");
      return false;
    }
  }
  std::string newValue = DataStoreItemConverter::itemListOfIntegerToString(value);
  bool res = setItemValueWorker(componentID, infoID, newValue, write);
  if (transactionId == 0)
  {
    unlock();
  }
  return res;
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::setItemValue(std::string componentID, std::string infoID, const std::list<std::string>& value, int transactionId)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  bool write = false;
  if (transactionId == 0)
  {
    lock();
    write = true;
  }
  else
  {
    if (transactionId != m_activeTransaction)
    {
      DS_SW_ERROR("ERROR: Someone pretends to be in a transaction, but did not start one");
      return false;
    }
  }
  std::string newValue = DataStoreItemConverter::itemListOfStringToString(value);
  bool res = setItemValueWorker(componentID, infoID, newValue, write);
  if (transactionId == 0)
  {
    unlock();
  }
  return res;
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::setItemValue(std::string componentID, std::string infoID, const std::list<bool>& value, int transactionId)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  bool write = false;
  if (transactionId == 0)
  {
    lock();
    write = true;
  }
  else
  {
    if (transactionId != m_activeTransaction)
    {
      DS_SW_ERROR("ERROR: Someone pretends to be in a transaction, but did not start one");
      return false;
    }
  }
  std::string newValue = DataStoreItemConverter::itemListOfBooleanToString(value);
  bool res = setItemValueWorker(componentID, infoID, newValue, write);
  if (transactionId == 0)
  {
    unlock();
  }
  return res;
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::setItemValue(std::string componentID, std::string infoID, const std::list<double>& value, int transactionId)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  bool write = false;
  if (transactionId == 0)
  {
    lock();
    write = true;
  }
  else
  {
    if (transactionId != m_activeTransaction)
    {
      DS_SW_ERROR("ERROR: Someone pretends to be in a transaction, but did not start one");
      return false;
    }
  }
  std::string newValue = DataStoreItemConverter::itemListOfDoubleToString(value);
  bool res = setItemValueWorker(componentID, infoID, newValue, write);
  if (transactionId == 0)
  {
    unlock();
  }
  return res;
}
//
//-------------------------------------------------------------------------------------------------
//
  
// Always set values in local file
// If item is not present in local file, but it is in default file, then add it to local
// If it is not present in both, it cannot be set.
bool DataStore::setItemValueWorker(std::string componentID, std::string infoID, std::string newValue, bool bWriteToFile)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  InfoKey infoKey = composeInfoKey(componentID, infoID);
  InfoMap::iterator iterInfoItems = m_localInfoMap->find(infoKey);
  
  if (iterInfoItems != m_localInfoMap->end()) // found in local?
  {
    // Update entry in lookup table.
    updateItem(iterInfoItems->second, newValue);
    
    if (bWriteToFile) // Write to a file, or are we busy executing a transaction?
    {
      std::string localInfoFile(m_infoFiles.at(iterInfoItems->second.category).localFile);
      // Write the updated XML tree to an XML file to persist it.
      if (writeToFile(localInfoFile, m_localXmlDocs.at(iterInfoItems->second.category)) != 0)
      {
        DS_SW_ERROR("ERROR: Could not write information to file %s", localInfoFile.c_str());
        return false;
      }
    }
    return true;
  }
  else
  {
    iterInfoItems = m_defaultInfoMap->find(infoKey);
    if (iterInfoItems != m_defaultInfoMap->end()) // found in default?
    {
      // add to local
      if (!addItemToLocal(iterInfoItems, newValue))
      {
        return false;
      }
       
      std::string localInfoFile(m_infoFiles.at(iterInfoItems->second.category).localFile);
      if (bWriteToFile)
      {
        if (writeToFile(localInfoFile, m_localXmlDocs.at(iterInfoItems->second.category)) != 0)
        {
          DS_SW_ERROR("ERROR: Could not write information to file %s", localInfoFile.c_str());
          return false;
        }
      }
      return true;
    }
  }
  
  // item not found
  logItemNotFound(componentID, infoID);
  return false;
}
//
//-------------------------------------------------------------------------------------------------
//
// First look in the local file, then in the default
bool DataStore::findItem(std::string componentID, std::string infoID, InfoData& item)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  lock();
  // Look in local items
  InfoKey infoKey = composeInfoKey(componentID, infoID);
  InfoMap::const_iterator iterInfoItems = m_localInfoMap->find(infoKey);
  if (iterInfoItems != m_localInfoMap->end()) // found?
  {
    item = iterInfoItems->second;
    unlock();
    return true;
  } 
  else 
  {
    // Look in default items
    iterInfoItems = m_defaultInfoMap->find(infoKey);
    if (iterInfoItems != m_defaultInfoMap->end()) // found?
    {
      item = iterInfoItems->second;
      unlock();
      return true;
    } 
  }
  
  unlock();
  // item not found
  logItemNotFound(componentID, infoID);
  return false;
}
//
//-------------------------------------------------------------------------------------------------
//
// Add an item that is present in default to local
bool DataStore::addItemToLocal(InfoMap::iterator infoKeyInDefault, std::string newValue)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  xmlNode* nodeInDefault = infoKeyInDefault->second.node;
  DS_LOG_GEN_TRACE("Copying %s", getID(nodeInDefault).c_str());
  
  // copy the node from the default to the local xml tree
  xmlNode* newNode = copyNodeFromDefaultToLocal(infoKeyInDefault->second.category, nodeInDefault);
  if (newNode == NULL)
  {
    return false;
  }
  // update the value in local
  xmlSetProp(newNode, (const xmlChar*)TAG_ITEMVALUE.c_str(), (const xmlChar*)newValue.c_str());
  
  // Add the new value to the local map
  xmlChar* temp = xmlGetProp(newNode, (const xmlChar*)TAG_ITEMTYPE.c_str());
  std::string infoType = (char*)temp;
  InfoData infoTypeAndValue = {infoKeyInDefault->second.category, newNode, infoType, newValue};
  InfoKey infoKey = infoKeyInDefault->first;
  m_localInfoMap->insert(InfoPair(infoKey, infoTypeAndValue));  
  return true;
}
//
//-------------------------------------------------------------------------------------------------
//
// recursive function to copy nodes
// returns the new node in local
// returns the equivalent node in local (either new or existing)
xmlNode* DataStore::copyNodeFromDefaultToLocal(int category, xmlNode* node)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  if (node == xmlDocGetRootElement(m_defaultXmlDocs.at(category)))
  {
    return xmlDocGetRootElement(m_localXmlDocs.at(category));
  } 
  else
  {
    // copy parent, then this.
    xmlNode *parentInLocal = copyNodeFromDefaultToLocal(category, node->parent);
    if (parentInLocal == NULL)
    {
      return NULL;
    }
    // copy this node
    bool found = false;
    // find if a corresponding node is present
    for (xmlNode* cur_node = parentInLocal->children; cur_node != NULL; cur_node = cur_node->next) 
    {
      if(cur_node->type == XML_ELEMENT_NODE)
      {
        if (getID(cur_node) == getID(node))
        {
          // found, return corresponding node in local
          found = true;
          return cur_node;
        }
      }
    } 
    if (!found)
    {
      // copy node
      xmlAddChild(parentInLocal, xmlNewText((const xmlChar*)"\n"));
      xmlNode* newNode = xmlNewNode(NULL, (const xmlChar*)node->name);
      xmlSetProp(newNode, (const xmlChar*)TAG_ID.c_str(), (const xmlChar*)getID(node).c_str());
      if (node->children == NULL)
      {
        xmlSetProp(newNode, (const xmlChar*)TAG_ITEMTYPE.c_str(), (const xmlChar*)getInfoType(node).c_str());
        xmlSetProp(newNode, (const xmlChar*)TAG_ITEMVALUE.c_str(), (const xmlChar*)getInfoValue(node).c_str());
      }
      xmlNode* child = xmlAddChild(parentInLocal, newNode);
      if (child == NULL) 
      {
        DS_SW_ERROR("ERROR: could not add xml node");
        return NULL;
      }
      xmlAddChild(parentInLocal, xmlNewText((const xmlChar*)"\n"));

      return child;
    }
  }
  DS_LOG_GEN_TRACE("Returning null, should not happen!");
  return NULL;
}
//
//-------------------------------------------------------------------------------------------------
//
void DataStore::updateItem(InfoData& item, std::string newValue)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return ;
  }

  // update internal map
  item.infoValue = newValue;
  
  // Update corresponding XML tree node.
  xmlNode* node = item.node;
  xmlSetProp(node, (const xmlChar*)TAG_ITEMVALUE.c_str(), (const xmlChar*)newValue.c_str());
  return;
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::addNewItem(std::string fileName, std::string componentID, std::string infoID, std::string newValue, int transactionId)
{
    return addNewItemWorker(fileName, componentID, infoID, newValue, InfoTypeString(), transactionId);
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::addNewItem(std::string fileName, std::string componentID, std::string infoID, int newValue, int transactionId)
{
    return addNewItemWorker(fileName, componentID, infoID, DataStoreItemConverter::itemIntegerToString(newValue), InfoTypeInt(), transactionId);
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::addNewItem(std::string fileName, std::string componentID, std::string infoID, double newValue, int transactionId)
{
    return addNewItemWorker(fileName, componentID, infoID, DataStoreItemConverter::itemDoubleToString(newValue), InfoTypeDouble(), transactionId);
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::addNewItem(std::string fileName, std::string componentID, std::string infoID, bool newValue, int transactionId)
{
    return addNewItemWorker(fileName, componentID, infoID, DataStoreItemConverter::itemBooleanToString(newValue), InfoTypeBool(), transactionId);
}
//
//=================================================================================================
//
bool DataStore::addNewItem(std::string fileName, std::string componentID, std::string infoID, std::list<std::string>& newValue, int transactionId)
{
    return addNewItemWorker(fileName, componentID, infoID, DataStoreItemConverter::itemListOfStringToString(newValue), InfoTypeListString(), transactionId);
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::addNewItem(std::string fileName, std::string componentID, std::string infoID, std::list<int>& newValue, int transactionId)
{
    return addNewItemWorker(fileName, componentID, infoID, DataStoreItemConverter::itemListOfIntegerToString(newValue), InfoTypeListInt(), transactionId);
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::addNewItem(std::string fileName, std::string componentID, std::string infoID, std::list<double>& newValue, int transactionId)
{
    return addNewItemWorker(fileName, componentID, infoID, DataStoreItemConverter::itemListOfDoubleToString(newValue), InfoTypeListDouble(), transactionId);
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::addNewItem(std::string fileName, std::string componentID, std::string infoID, std::list<bool>& newValue, int transactionId)
{
    return addNewItemWorker(fileName, componentID, infoID, DataStoreItemConverter::itemListOfBooleanToString(newValue), InfoTypeListBool(), transactionId);
}
//
//=================================================================================================
//
bool DataStore::addNewItemWorker(std::string fileName, std::string componentID, std::string infoID, std::string newValue, std::string itemType, int transactionId) 
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  InfoKey infoKey = composeInfoKey(componentID, infoID);
  bool write = false;
  if (transactionId == 0) 
  {
    lock();
    write = true;
  } 
  else 
  {
    if (transactionId != m_activeTransaction) 
    {
      DS_SW_ERROR("ERROR: Someone pretends to be in a transaction, but did not start one");
      return false;
    }
  }
  
  int categoryNr(0);
  bool fileFound(false);
  for (InfoFilesVector::const_iterator infoFilestIter = m_infoFiles.begin(); 
       ((infoFilestIter != m_infoFiles.end()) && !fileFound);  
       ++infoFilestIter)
  {    
    fileFound = (fileName.compare(infoFilestIter->localFile) == 0);
    if (!fileFound) ++categoryNr;
  }
  
  if (!fileFound)
  {
    if (transactionId == 0)
    {
      unlock();
    }
    DS_LOG_GEN_TRACE("Cannot find file \"%s\" in administration.", fileName.c_str());
    return false;
  }
  
  InfoMap::const_iterator iterInfoItems = m_localInfoMap->find(infoKey);
  if (iterInfoItems != m_localInfoMap->end()) // found?
  {
    if (transactionId == 0)
    {
      unlock();
    }
    DS_LOG_GEN_TRACE("Cannot add item \"%s\" to file \"%s\". It already exists.", infoKey.c_str(), fileName.c_str());
    return false;
  }

  // find the component node
  std::string temp = "";
  xmlNode *root = xmlDocGetRootElement(m_localXmlDocs.at(categoryNr));
  xmlNode *component = findComponent(componentID, root, temp);

  // If the component does not yet exist, add it.
  if (component == NULL) 
  {
    addNewComponent(categoryNr, componentID);
    component = findComponent(componentID, root, temp);
    // add newline 
    xmlAddChild(root, xmlNewText((const xmlChar*)"\n"));
  }
  
  // add item node
  // add newline 
  xmlAddChild(component, xmlNewText((const xmlChar*)"\t"));

  xmlNode *newNode = xmlNewNode(NULL, (const xmlChar*)TAG_ITEM.c_str());
  xmlSetProp(newNode, (const xmlChar*)TAG_ID.c_str(), (const xmlChar*)infoID.c_str());
  xmlSetProp(newNode, (const xmlChar*)TAG_ITEMTYPE.c_str(), (const xmlChar*)itemType.c_str());
  xmlSetProp(newNode, (const xmlChar*)TAG_ITEMVALUE.c_str(), (const xmlChar*)newValue.c_str());
  xmlNode* child = xmlAddChild(component, newNode);
  
  if (child == NULL) 
  {
    if (transactionId == 0)
    {
      unlock();
    }
    DS_SW_ERROR("ERROR: could not add xml node");
    return false;
  }

    // Add the new value to the local map
  InfoData infoTypeAndValue = {categoryNr, child, itemType.c_str(), newValue};
  m_localInfoMap->insert(InfoPair(infoKey, infoTypeAndValue));  

  // add newline 
  xmlAddChild(component, xmlNewText((const xmlChar*)"\n"));

  // write to file
  if (m_activeTransaction == 0)
  {
    if (writeToFile(fileName, m_localXmlDocs.at(categoryNr)) != 0)
    {
      if (transactionId == 0)
      {
        unlock();
      }
      DS_SW_ERROR("ERROR: Could not write information to file \"%s\"", fileName.c_str());
      return false;
    }
  }
  
  if (transactionId == 0)
  {
    unlock();
  }
  return true;
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::addNewComponent(int category, std::string componentID) 
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  std::string temp = "";
  xmlNode *root = xmlDocGetRootElement(m_localXmlDocs.at(category));
  xmlNode *component = findComponent(componentID, root, temp);

  if (component != NULL)
  {
    DS_LOG_GEN_TRACE("Cannot add new %s %s. It already exists.", m_tag_component.c_str(), componentID.c_str());
    return false;
  }
  
  xmlNode *newComponentNode = xmlNewNode(NULL, (const xmlChar*)m_tag_component.c_str());
  xmlSetProp(newComponentNode, (const xmlChar*)TAG_ID.c_str(), (const xmlChar*)componentID.c_str());
  xmlNode* child = xmlAddChild(root, newComponentNode);
  if (child == NULL) 
  {
    DS_SW_ERROR("ERROR: could not add %s node", m_tag_component.c_str());
    return false;
  }
  xmlAddChild(child, xmlNewText((const xmlChar*)"\n"));

  // write result to file
  if (m_activeTransaction == 0)
  {
    if (writeToFile(m_infoFiles.at(category).localFile, m_localXmlDocs.at(category)) != 0)
    {
      DS_SW_ERROR("ERROR: Could not write information to file %s", m_infoFiles.at(category).localFile.c_str());
      return false;
    }
  }
  
  return true;
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::removeItem(std::string componentID, std::string infoID, int transactionId)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  InfoKey infoKey = composeInfoKey(componentID, infoID);
  
  bool write = false;
  if (transactionId == 0)
  {
    lock();
    write = true;
  }
  else
  {
    if (transactionId != m_activeTransaction)
    {
      DS_SW_ERROR("ERROR: Someone pretends to be in a transaction, but did not start one");
      return false;
    }
  }
  
  InfoMap::iterator iterInfoItems = m_localInfoMap->find(infoKey);
  if (iterInfoItems == m_localInfoMap->end()) // found?
  {
    if (transactionId == 0)
    {
      unlock();
    }
    DS_LOG_GEN_TRACE("Cannot remove item %s. It does not exist (in the local xml file).", infoKey.c_str());
    return false;
  }
  
  std::string localInfoFile(m_infoFiles.at(iterInfoItems->second.category).localFile);
  
  // remove from xml tree
  xmlNode *node = iterInfoItems->second.node;
  xmlNode *next = node->next;
  xmlUnlinkNode(node);
  xmlFreeNode(node);
  if (next->type == XML_TEXT_NODE) 
  {
    // remove newline
    xmlUnlinkNode(next);
    xmlFreeNode(next);
  }   
  
  // remove from memory map
  m_localInfoMap->erase(iterInfoItems);
  
  // write to file
  if (m_activeTransaction == 0)
  {
    if (writeToFile(localInfoFile, m_localXmlDocs.at(iterInfoItems->second.category)) != 0)
    {
      if (transactionId == 0)
      {
        unlock();
      }
      DS_SW_ERROR("ERROR: Could not write information to file %s", localInfoFile.c_str());
      return false;
    }
  }
  
  if (transactionId == 0)
  {
    unlock();
  }
  return true;
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::removeComponent(std::string componentID, int transactionId)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  bool write = false;
  if (transactionId == 0)
  {
    lock();

    write = true;
  }
  else
  {
    if (transactionId != m_activeTransaction)
    {
      DS_SW_ERROR("ERROR: Someone pretends to be in a transaction, but did not start one");
      return false;
    }
  }

  // remove all items and sub items from the map
  // remove all nodes from the tree
  // easiest way: throw away map and build it up again. 
  // (ok, it is not the nicest way)
  m_localInfoMap->clear();


  int categoryNr(0);
  for (InfoFilesVector::const_iterator infoFilestIter = m_infoFiles.begin(); 
       infoFilestIter != m_infoFiles.end();  
       ++infoFilestIter)
  {    
    // find the component in the tree (using linear search, which is maybe a bit naive)
    xmlNode *root = xmlDocGetRootElement(m_localXmlDocs.at(categoryNr));
    
    std::string temp = "";
    xmlNode *component = findComponent(componentID, root, temp);
  
    if (component == NULL) 
    {
      if (transactionId == 0)
      {
        unlock();
      }
      DS_SW_ERROR("ERROR: Could not remove %s %s. It does not exist.", m_tag_component.c_str(), componentID.c_str());
      return false;
    }
  
    xmlUnlinkNode(component);
    xmlFreeNode(component); // this is a recursive free

    
    // also do this in case of a transaction, because we cannot update 
    // our local map without rereading it after a write
    writeToFile(infoFilestIter->localFile, m_localXmlDocs.at(categoryNr));
    xmlDoc* newLocalXmlDoc = NULL;
    loadFromFile(infoFilestIter->localFile, categoryNr, newLocalXmlDoc, m_localInfoMap);
    // remove m_localXmlDocs.at(categoryNr)
    m_localXmlDocs.at(categoryNr) = newLocalXmlDoc; // store the new xml tree at the same place
    ++categoryNr;
  }

  if (transactionId == 0)
  {
    unlock();
  }
  return true;
}
//
//-------------------------------------------------------------------------------------------------
//
// recursively find the node in the xml tree that has componentID, currentComponentID gradually builds up the recursive componentID
xmlNode *DataStore::findComponent(std::string componentID, xmlNode *node, std::string currentComponentID)
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return false;
  }

  if (node->type == XML_ELEMENT_NODE)
  {
    // ignore information tag
    if ((char*)node->name != TAG_INFORMATION)
    {
      // compose recursive componentID
      std::string id = getID(node);
      if (currentComponentID == "") 
      {
        currentComponentID = id;
      } 
      else 
      {
        currentComponentID = currentComponentID + "." + id;
      }
//      DS_LOG_GEN_TRACE("%s -> %s", getID(node).c_str(), currentComponentID.c_str());
    }
    
    // Check if this is the node you are looking for
    if (((char*)node->name == m_tag_component) && (currentComponentID == componentID))
    {
      return node;
    }
    else // if not, try your child nodes
    {
      for (xmlNode* cur_node = node->children; cur_node != NULL; cur_node = cur_node->next) 
      {
        node = findComponent(componentID, cur_node, currentComponentID);
        if (node != NULL) 
        {
          return node;
        }
      } // for
    }
  }
  else
  {
    return NULL;
  }
  return NULL;
} 
//
//-------------------------------------------------------------------------------------------------
//
/// @todo Return false instead of empty list in case of error. 
std::list<std::string> DataStore::getRootComponents(const std::string& file)
{
  std::list<std::string> components;

  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return components;
  }

  lock();
 
  int categoryNr(0);
  bool fileFound(false);
  for (InfoFilesVector::const_iterator infoFilestIter = m_infoFiles.begin(); 
       ((infoFilestIter != m_infoFiles.end()) && !fileFound);  
       ++infoFilestIter)
  {    
    fileFound = (file.compare(infoFilestIter->localFile) == 0);
    if (!fileFound) ++categoryNr;
  }
  
  if (static_cast<unsigned int>(categoryNr) >= m_localXmlDocs.size())
  {
    DS_LOG_GEN_TRACE("ERROR: A file was specified which wasn't supplied at initialization");
    unlock();
    return components; // which is still empty at this point
  }

  xmlNode *root = xmlDocGetRootElement(m_localXmlDocs.at(categoryNr));

  for (xmlNode* cur_node = root->children; cur_node != NULL; cur_node = cur_node->next) 
  {
    if (cur_node->type == XML_ELEMENT_NODE)
    {
      if ((char*)cur_node->name == m_tag_component) // == overloaded for std::string comparison
      {
        std::string componentID = getID(cur_node);
        components.push_back(componentID);
      }
    }
  } // for

  unlock();
  return components;
}
//
//-------------------------------------------------------------------------------------------------
//
std::set<std::string> DataStore::getSubItemsFromLocal(std::string componentID, std::string itemSetID)
{
  std::set<std::string> subItems;

  lock();

  InfoKey infoKey = composeInfoKey(componentID, itemSetID);

  int findCount = 0;
  BOOST_FOREACH(const InfoPair& ip, *m_localInfoMap)
  {
    if (boost::starts_with(ip.first, infoKey))
    {
      findCount++;
      
      // parse the key (consisting of hierarchic itemID's seperated by dots)
      
      // first remove the current key from the std::string that was found
      std::string tempString = boost::erase_first_copy(ip.first, infoKey); // erase_first_copy erases first occurrence from input and returns a copy of result
      
      // remove a possible leading dot (which was not part of the current key)
      boost::trim_left_if(tempString, boost::is_any_of("."));
      
      // split the remaining std::string into its constituents, as we are only interested in itemID's directly below the requested itemID
      std::vector<std::string> tempVector;
      boost::split(tempVector, tempString, boost::is_any_of("."));
      
      // add the found itemID to our set (so we keep only unique itemID's)
      if (!tempVector.empty())
      {
        subItems.insert(tempVector.front());
      }
    }
  }
  
  unlock();
  
  return subItems;
}
//
//-------------------------------------------------------------------------------------------------
//
void DataStore::logItemNotFound(std::string componentID, std::string itemID)
{
  DS_LOG_GEN_TRACE("Warning: Information item not found, %s id: %s, item id: %s",
                m_tag_component.c_str(), componentID.c_str(), itemID.c_str());
}
//
//-------------------------------------------------------------------------------------------------
//
// thread safety
void DataStore::lock() 
{
  pthread_mutex_lock(&m_dataLock);
}
//
//-------------------------------------------------------------------------------------------------
//
void DataStore::unlock()
{
  pthread_mutex_unlock(&m_dataLock);
}
//
//-------------------------------------------------------------------------------------------------
//
int DataStore::startAtomicTransaction()
{
  if (!m_initialized)
  {
    DS_SW_ERROR("DataStore not initialized");
    return -1;
  }

  lock();
  m_activeTransaction = generateTransactionId();
  return m_activeTransaction;
}
//
//-------------------------------------------------------------------------------------------------
//
bool DataStore::endAtomicTransaction(int id)
{
  if (m_activeTransaction == 0 || m_activeTransaction != id)
  {
    DS_SW_ERROR("ERROR: Someone tries to end a transaction with the wrong transaction id");
    return false;
  }
  // write all local information files to disk
  if (m_localXmlDocs.size() != m_infoFiles.size()) 
  {
    DS_SW_ERROR("ERROR: m_localXmlDocs size: %d, m_infoFiles size: %d",
             (int)m_localXmlDocs.size(), (int)m_infoFiles.size());
    return false;
  }
  assert(m_localXmlDocs.size() == m_infoFiles.size());
  
  std::vector<xmlDoc*>::iterator itDocs= m_localXmlDocs.begin();
  for (InfoFilesVector::iterator it = m_infoFiles.begin(); it != m_infoFiles.end(); it++)
  {
    std::string localInfoFile = it->localFile;
    xmlDoc *doc = *itDocs;
    if (writeToFile(localInfoFile, doc) != 0)
    {
      DS_SW_ERROR("ERROR: Could not write information to file %s", localInfoFile.c_str());
      unlock();
      return false;
    }
    
    itDocs++;
  }
  m_activeTransaction = 0;
  unlock();
  return true;
}
//
//-------------------------------------------------------------------------------------------------
//
int DataStore::generateTransactionId()
{
  static int id = 0;
  id++;
  // counts from 1 to max_int. 0 is reserved for "no transaction"
  if (id == std::numeric_limits<int>::max())
  {
    id = 1;
  }
  return id;  
}

