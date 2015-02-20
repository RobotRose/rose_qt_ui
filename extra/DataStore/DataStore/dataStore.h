#ifndef DATASTORE_H
#define DATASTORE_H

#include <iostream>
#include <map>
#include <list>
#include <set>
#include <string>

#include "dataStoreIf.h"

#include "ClientLogger.h"
#include "InfoFilesVector.h"

/** Forward declarations, definition will be in the libxml2 header file */
struct _xmlNode;
typedef struct _xmlNode xmlNode;

struct _xmlDoc;
typedef struct _xmlDoc xmlDoc;

typedef std::string InfoKey;

/// Used to keep an in-memory copy of the xml file(s)
struct InfoData
{
  int category; ///< The category of the item
  xmlNode* node; ///< Reference to the node in the xmlTree
  std::string infoType; ///< type of the item
  std::string infoValue; ///< value of the item
};

typedef std::pair<InfoKey, InfoData> InfoPair;
typedef std::map<InfoKey, InfoData> InfoMap; ///< The in-memory representation of an xml file

// Separators used for constructing a key from multiple std::strings. See services design document for explanation
static const std::string KEY_STRING_SEPARATOR = ";";
static const std::string SEPARATOR = ".";

/**
* The datastore class can be used in 3 different ways:
* 1. Initialize the datastore with two file. One will be used to read default values from, the other is used to
*    overwrite these defaults with new values. The default file is never changed. Updating, adding and deleting
*    information is always done on the local information file. Deleting an item that also exists in the default 
*    file implicitly restores the default value (it is never deleted from the default file).
* 2. If the local information filename that is specified, does not exist, an empty local file is created and
*    used (using the specified file name).
* 3. Initialize the datastore with only one file. No default values are used in this case. 
*
* An info item is identified by its componentID and itemID, which are concatenated using a separation character
*/
class DataStore : public DataStoreIf
{
private:
  /// After initialization, the data store is thread-safe. All data access is locked by this mutex.
  pthread_mutex_t m_dataLock;

  /// if in a transaction, changes are not written to file.
  /// if m_activeTransaction == 0, no transaction is active, otherwise, m_activeTransaction == the current
  /// transactionId. 
  int m_activeTransaction;
  
  bool m_initialized;

  /** The in-memory XML representation of the default information sets. 
  */
  std::vector<xmlDoc*> m_defaultXmlDocs; // TODO This should be read-only
  
  /** The in-memory XML representation of the local information sets.
  */
  std::vector<xmlDoc*> m_localXmlDocs;

  /** The information XML files. */
  InfoFilesVector m_infoFiles;

  /** The component tag */
  std::string m_tag_component;

  /** A lookup table containing the type and value of information items in the default file. The key in the table is the module ID, component ID and info ID. */
  InfoMap *m_defaultInfoMap;

  /** A lookup table containing the type and value of information items in the local file. The key in the table is the module ID, component ID and info ID. */
  InfoMap *m_localInfoMap;
  
public:
  /** Constructor.
   * @param compontentTag the tags used for items at the root of the XML file to read
   */
  DataStore(std::string componentTag);
  
  /// Destructor
  ~DataStore();

  /**
  * The initialize function should always be called before using the datastore.
  * Initialize the datastore with two files. If the localInfo file does not exist, a new (empty) one is created.
  * <b>During initialization, all info items that are in local, but not in default are removed!!</b>
  * @param defaultInfo Default information item values
  * @param localInfo Local changes on the defaults. This file is adapted when setting item values
  * @return 0 if succeeded, 1 otherwise
  */
  int initialize(std::string defaultInfo, std::string localInfo);
  
  /** 
  * Initialize function to use datastore with only one file. 
  * In this case no default values are used. 
  * @param localInfo information file
  * @return 0 if succeeded, 1 otherwise
  */
  int initialize(std::string localInfo);

  /**
   * Initialize function for initialization with multiple files.
   * @param informationFiles should contain pairs of default and local files.
   * @return 0 if succeeded, 1 otherwise
   */
  int initialize(InfoFilesVector& informationFiles);

  //   get or set items (public interface)
  /**
  * @param componentID the id's used for items at the root of the XML file
  * @param infoID the id's used for items within componentID's
  * @param value Output parameter. Returns the local value or, in case the item does not exist in local, the default value.
  * @return whether the retrieval of a value succeeded.
  */
  // get primitives
  bool getItemValue(std::string componentID, std::string infoID, std::string& value);
  bool getItemValue(std::string componentID, std::string infoID, int& value);
  bool getItemValue(std::string componentID, std::string infoID, bool& value);
  bool getItemValue(std::string componentID, std::string infoID, double& value);
  
  // get lists
  bool getItemValue(std::string componentID, std::string infoID, std::list<std::string>& value);
  bool getItemValue(std::string componentID, std::string infoID, std::list<int>& value);
  bool getItemValue(std::string componentID, std::string infoID, std::list<bool>& value);
  bool getItemValue(std::string componentID, std::string infoID, std::list<double>& value);
  
    
  // set primitives
  /// @param transactionId 0 if not part of a transaction
  bool setItemValue(std::string componentID, std::string infoID, const std::string& value, int transactionId = 0);
  bool setItemValue(std::string componentID, std::string infoID, const int value, int transactionId = 0);
  bool setItemValue(std::string componentID, std::string infoID, const bool value, int transactionId = 0);
  bool setItemValue(std::string componentID, std::string infoID, const double value, int transactionId = 0);

  // set lists
  bool setItemValue(std::string componentID, std::string infoID, const std::list<std::string>& value, int transactionId = 0);
  bool setItemValue(std::string componentID, std::string infoID, const std::list<int>& value, int transactionId = 0);
  bool setItemValue(std::string componentID, std::string infoID, const std::list<bool>& value, int transactionId = 0);
  bool setItemValue(std::string componentID, std::string infoID, const std::list<double>& value, int transactionId = 0);
  
 
public:
  // more advanced public interface

  /** Add an item (that is not specified in default) to the local xml file. 
  * If the componentID is already in the file, the item is added at this place. If not, a new component is added.
  * @return succeeded
  * @param id should be set to the transaction id if this call is part of a transaction
  */
  bool addNewItem(std::string fileName, std::string componentID, std::string infoID, std::string newValue, int transactionId = 0);
  bool addNewItem(std::string fileName, std::string componentID, std::string infoID, int newValue, int transactionId = 0);
  bool addNewItem(std::string fileName, std::string componentID, std::string infoID, double newValue, int transactionId = 0);
  bool addNewItem(std::string fileName, std::string componentID, std::string infoID, bool newValue, int transactionId = 0);

  // add lists
  bool addNewItem(std::string fileName, std::string componentID, std::string infoID, std::list<std::string>& newValue, int transactionId = 0);
  bool addNewItem(std::string fileName, std::string componentID, std::string infoID, std::list<int>& newValue, int transactionId = 0);
  bool addNewItem(std::string fileName, std::string componentID, std::string infoID, std::list<double>& newValue, int transactionId = 0);
  bool addNewItem(std::string fileName, std::string componentID, std::string infoID, std::list<bool>& newValue, int transactionId = 0);

  /** To store a set of items at once, a transaction can be used. 
   * Important: all calls that take part in the transaction have to be from 1 thread.
   * A transaction is started by startAtomicTransaction. An id is returned which is then used to 
   * identify the transaction (and prevent race conditions)
   * then a number of items can be set, added, or removed (not read/get) using this id, 
   * Warning: If the id is not passed, a deadlock occurs.
   * then the transaction is ended (and the data written to file) by calling endAtomicTransaction
   * During a transaction the datastore blocks all other incoming requests (including reads)
   * @return transaction id.
   */
  int startAtomicTransaction();
  
  bool endAtomicTransaction(int id);
    
  /** 
   * Remove an item from local information. This restores the default value in case there is one.
   * @return succeeded
   */
  bool removeItem(std::string componentID, std::string infoID, int transactionId = 0);

  /**
   * Remove a component and all nested items and components
   * @return succeeded
   */
  bool removeComponent(std::string componentID, int transactionId = 0);

  /**
   * Get list of all components directly under root of specified file. Since the DataStore may be
   * initialized with multiple files, you must specify which file you want the root components of.
   */
  std::list<std::string> getRootComponents(const std::string& file);

  /**
   * Get list of all components directly under some root component or itemset.
   */
  std::set<std::string> getSubItemsFromLocal(std::string componentID, std::string itemSetID);

private:
  // constants functions
  static std::string InfoTypeString()     { return "string"; }
  static std::string InfoTypeInt()        { return "int"; }
  static std::string InfoTypeDouble()     { return "double"; }
  static std::string InfoTypeBool()       { return "bool"; }
  static std::string InfoTypeListString() { return "list_string"; }
  static std::string InfoTypeListInt()    { return "list_int"; }
  static std::string InfoTypeListDouble() { return "list_double"; }
  static std::string InfoTypeListBool()   { return "list_bool"; }

  // add functions worker memeber
  // If the componentID is already in the file, the item is added at this place. If not, a new component is added.
  // @return succeeded
  // @param id should be set to the transaction id if this call is part of a transaction
  bool addNewItemWorker(std::string fileName, std::string componentID, std::string infoID, std::string newValue, std::string itemType, int transactionId);

  /**
   * The new value is written in the local file. 
   * @param newValue Use the information type parameters to convert the value to a std::string.
   * @param bWriteToFile The information file is updated if bWriteToFile=true; Otherwise, 
   * the value is only updated in the map (not persistent)
   */
  bool setItemValueWorker(std::string componentID, std::string infoID, std::string newValue, bool bWriteToFile = true);
  
  /** Add a component with no items.
  * @return returns false if the component already exists.
  */
  bool addNewComponent(int category, std::string componentID);

  // helper functions
  /** Find an item in the map
  */
  bool findItem(std::string componentID, std::string infoID, InfoData& item);

  /** Set a new value
  * @param item Can be found using findItem
  */
  void updateItem(InfoData& item, std::string newValue);
  
  /**
  * Check if all values in the localInformationFile are also present in the defaultInformation file. If not, remove them from the local file.
  */
  void checkConsistency();

  /** Add an item to the local file that is also specified in the default file (but not yet in local). 
  * This is done by first copying the item from the default file to the local file (and map, and xmlDoc) and then updating its value.
  */
  bool addItemToLocal(InfoMap::iterator infoKeyInDefault, std::string newValue);

  /**
  * Copy the node from the default file to the local file (recursive function)
  * @param node Pointer to the node to be copied. This node should be in the default file.
  */
  xmlNode* copyNodeFromDefaultToLocal(int category, xmlNode* node);
  
  /**
  *  Write information items to an XML file.
  * @param filename The file it should be written to
  * @param xmlDoc The doc that should be written to the file (The two should correspond)
  *  @return 0 - XML file written successfully. Any other value indicates an error.
  */
  int writeToFile(std::string filename, xmlDoc *xmldoc);

  /** @param componentID
  * @param node in root node
  * @param currentComponentID Needed for recursive calls. Should always be "" when  calling this function from outside.
  * @return NULL if component could not be found. Otherwise, a pointer to the node for that component.
  */
  xmlNode *findComponent(std::string componentID, xmlNode *node, std::string currentComponentID);
  
  int generateTransactionId();
  
  // helpers for easy locking and unlocking.
  void lock();
  void unlock();
 
  /**
  *  Load information items from XML file.
  * @param filename File to load from
  * @param xmlDoc The pointer is changed in this function. Output is the resulting xml tree
  * @param map with the information in the tree
  * @return 0 - XML file loaded successfully. Any other value indicates an error.
  *
  * Fail-safe persistent storage mechanism: 
  * when writing a file, do the following:
  * - Write to temp file
  * - sync() to make sure that the file is actually written to disk
  * - write a lock file
  * - sync
  * - remove original file
  * - sync
  * - rename the temp file to the original file
  * - sync
  * - remove the lock file
  *
  * In a normal situation (no errors), when writing the file, only the original file is present. No lock or temp file is present.
  */
  int loadFromFile(std::string filename, int category, xmlDoc*& xmlDoc, InfoMap* map);
  
  /**
  *  Recover the XML file in case the last write action has been interrupted.
  * This function brings the system in a status where only the original file is present, no lock or temp file. 
  *  @return 0 - The xml file has been successfully recovered.
  */
  int recoverFile(std::string filename);
  
  /**
  *  Check if the lock file exists.
  *  @return true - Lock file exists. False, otherwise.
  */
  bool lockFileExists(std::string filename) const;
  
  /**
  *  Check if the temporary file exists.
  *  @return true - Temporary file exists. False, otherwise.
  */
  bool tempFileExists(std::string filename) const;
  
  /**
  *  Try to remove the original XML file.
  *  @return 0 - Original XML file successfully removed. Any other value indicates an error.
  */
  int removeOriginalFile(std::string filename);
  
  /**
  *  Check if the original file exists.
  *  @return true - Original file exists. False, otherwise.
  */
  bool originalFileExists(std::string filename) const;
  
  /**
  *  Try to write the lock file.
  *  @return 0 - Lock file successfully written. Any other value indicates an error.
  */
  int writeLockFile(std::string filename);
  
  /**
  *  Try to remove the lock file.
  *  @return 0 - Lock file successfully removed. Any other value indicates an error.
  */
  int removeLockFile(std::string filename);
  
  /**
  *  Try to remove the temporary file.
  *  @return 0 - Temporary file successfully removed. Any other value indicates an error.
  */
  int removeTempFile(std::string filename);
  
  /**
  *  Try to rename the temporary file to the original file.
  *  @return 0 - Temporary file successfully renamed. Any other value indicates an error.
  */
  int renameTempFile(std::string filename);
  

  // XML helper methods.
  
  /**
  *  Load <information> section of the input file.
  *  @param node - The root node of the <information> section.
  * @param map - Map in which the information should be stored
  */
  int readInformation(int category, xmlNode* node, InfoMap *map);

  /**
  *  Load <component> section of the input file.
  *  @param node - The root node of the <component> section.
  *  @param moduleID - The parent module ID (as std::string).
  * @param map - Map in which the information should be stored
  */
  int readComponent(int category, xmlNode* node, std::string moduleID, InfoMap *map);

  /**
  *  Load <info> section of input file.
  *  @param node - The root node of the <info> section.
  *  @param moduleID - The parent module ID (as std::string).
  *  @param componentID - The parent component ID (as std::string).
  * @param map - Map in which the information should be stored
  */
  int readItemset(int category, xmlNode* node, const std::string componentID, std::string itemID, InfoMap *map);
  
  /**
  *  Load <info> section of input file.
  *  @param node - The root node of the <info> section.
  *  @param moduleID - The parent module ID (as std::string).
  *  @param componentID - The parent component ID (as std::string).
  * @param map - Map in which the information should be stored
  */
  void readInfo(int category, xmlNode* node, const std::string& componentID, const std::string itemID, InfoMap *map);

  /**
  *  Get the 'id' value of the XML node.
  *  @param node - The XML node to read the attribute for.
  *  @return The 'id' attribute value of the XML node.
  */
  std::string getID(xmlNode* node);

  /**
  *  Get the 'type' value of the XML node.
  *  @param node - The XML node to read the attribute for.
  *  @return The 'type' attribute value of the XML node.
  */
  std::string getInfoType(xmlNode* node);

  /**
  *  Get the 'value' value of the XML node.
  *  @param node - The XML node to read the attribute for.
  *  @return The 'value' attribute value of the XML node.
  */
  std::string getInfoValue(xmlNode* node);
    
  /**
  *  Compose a key for the information items lookup table using the specified module ident, component ident and info ident.
  *  @return The key (std::string) used in the lookup table for information items.
  */
  InfoKey composeInfoKey(
    const std::string& componentID,
    const std::string& infoID);
    
  /**
   * Log the fact that the information item is not found/defined for the specified component ID and info ID.
   */
  void logItemNotFound(std::string componentID, std::string itemID);
    
};

#endif
