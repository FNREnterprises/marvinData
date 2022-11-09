
# How to Add New Shared Objects

marvinData in combination with marvinglobal instanciates the shared objects in the marvin environment
To allow processes to see the shared data objects the following setup is required:

in the main section of the process try to connect with marvinData

from marvinglobal import marvinShares

from marvinglobal import marvinglobal as mg

config.marvinShares = marvinShares.MarvinShares()

once the process has successfully set up a reference to the shared object it can 
- dict read values 
    headImu = config.marvinShares.cartDict.get(mg.SharedDataItems.HEAD_IMU)
        as in the above case cartDict is a DictProxy object (python 3.9) we need to use the get() method
        to access subDicts.
  - dict update values
        for updates we need to have a local copy of the object as the shared dicts can not directly be modified
        update the local dict and use the dicts publish method you should have implemented (see below section
        how to add a shared dict)
    - queue get items
      request = config.marvinShares.<queue>.get()
    - queue put items
      config.marvinShares.<queue>.put(<data>)

in order to prevent processes getting stuck on waiting for requests and terminate gracefully 
in case of a connection loss the get call will throw an exception
the process needs to check periodically connection with marvinData and not wait endlessy
request = config.marvinShares.imageProcessingRequestQueue.get(block=True, timeout=1)

# To add a shared dict (example NEW_DICT):
- marvinglobal.py
  
    add a new dict type into SharedDataItems list and assign a unique number to it
    SharedDataItems
        NEW_DICT = "unique number"
    The number is used when receiving update requests to update the meant dict  
  
 - marvinData.py
   
    class MarvinData
   
        __init_
            create the shared dict in the MarvinData class
                self.newDict = ShareManager.dict()
                in case of a subDict
                    parentDict.update({mg.SharedDataItems.NEW_DICT: config.newDictNew})

        create a get method for the dict
            def getNewDict(self):
                return self.newDict

        run
            register the get method
            ShareManager.register('getNewDict', self.getNewDict)
                    
    config.py
        add a local version of the dict as a config variable
            processes modify the local dict first, then send it to marvinData for updating the shared dict
    MarvinData.sharedDataUpdateRequests
        add a section for updating the shared dict
<owner>.py
    create a local version of the dict in the config.py
    add a publish method for the dict
    the <owner> process needs to have a local version of the dict in its config file.
    on changes of any value in the dict the owner publishes the local dict to marvinData
    marvinData will then update its shared version of the dict and this will allow other processes to see the changes
        
      
##To add a shared queue:

-   class marvinglobal.MarvinShares
    
    `__init__` 
    register the get method
    
    `ShareManager.register("getMainGuiUpdateQueue") `

    sharedDataConnect  
    
    conditionally add the queue
    
    `    if 'CartRequestQueue' in ressourceList:
            try:
                self.cartRequestQueue = self.m.getCartRequestQueue()
                mg.log(f"cartRequestQueue available")
            except Exception as e:
                mg.log(f"could not get access to cartRequestQueue, {e}")
                return False
    `

-   marvinglobal.usedSharedRessources
    add the queue to all processes needing it

-   marvinData
    
    `__init__` section, 
    
    add the queue to class MarvinData
    e.g. self.cartRequestQueue = multiprocessing.Queue()

    register the queue in the class MarvinData in the run module

in modules accessing shared elements use the following imports:
    from marvinglobal import marvinglobal as mg
    from marvinglobal import marvinShares
    from marvinglobal import cartClasses
    from marvinglobal import CartCommandMethods
    from marvinglobal import servoClasses
    from marvinglobal import servoCommandMethods