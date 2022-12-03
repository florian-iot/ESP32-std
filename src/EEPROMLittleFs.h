#include "CompilationOpts.h"

#ifdef USE_EEPROM_LFS
#ifndef INCL_EEPROM_LITTLE_FS
#define INCL_EEPROM_LITTLE_FS


#include <limits>
#include <stddef.h>
#include <FS.h>
#include <FSImpl.h>
#include <debug.h>
#include <lfs.h>
#include "EEPROMDriver.h"

// #define EEPROM_LFS_IN_MEMORY

class EEPROMLittleFsImpl;

class EEPROMLittleFs: public fs::FS {
public:
    struct I2CConfig {
        int sdaPin = -1;
        int sclPin = -1;
        int freqHz = 100000;
        int port = 0;
    };
    struct EEPROMConfig {
        uint8_t deviceAddress = 0x50;
        size_t size = -1;
        size_t writePageSize = -1;
    };
    struct LFSConfig {
        size_t block_size = 128;
        int block_cycles = 500;
        int cache_size = 128;
        int lookahead_size = 64;
    };

    EEPROMLittleFs();
    bool init(const I2CConfig &i2cCfg, const EEPROMConfig &eeCfg, const LFSConfig &lfsCfg, bool allowFormatting);
    bool init(int i2cPort, const EEPROMConfig &eeCfg, const LFSConfig &lfsCfg, bool allowFormatting);
    void terminate();

    // If the file system was already mounted, it is mounted again after formatting
    bool format();
    // Last filesystem error
    void getLastError(String *msg);
    // Last driver error
    void getDriverLastError(String *msg);

private:
    EEPROMLittleFsImpl *fsImplPtr;
};


class EEPROMLittleFsImpl: public fs::FSImpl {
private:
    lfs_t lfs;
    struct lfs_config cfg;
    int lastError;
    bool isInitialized;
#ifdef EEPROM_LFS_IN_MEMORY
    uint8_t *diskSpace;
#else
    EEPROMMasterDriver eeDriver;
#endif

    friend int eepromLfsRead(const struct lfs_config *c, lfs_block_t block,
        lfs_off_t off, void *buffer, lfs_size_t size);
    friend int eepromLfsProg(const struct lfs_config *c, lfs_block_t block,
        lfs_off_t off, const void *buffer, lfs_size_t size);
    friend int eepromLfsErase(const struct lfs_config *c, lfs_block_t block);
    friend int eepromLfsSync(const struct lfs_config *c);

    const char *errorStr(int rc);

    int mount(bool allowFormatting);

public:
    bool init(const EEPROMLittleFs::I2CConfig &i2cCfg, const EEPROMLittleFs::EEPROMConfig &eeCfg,
        const EEPROMLittleFs::LFSConfig &lfsCfg, bool allowFormatting);
    bool init(int i2cPort, const EEPROMLittleFs::EEPROMConfig &eeCfg, const EEPROMLittleFs::LFSConfig &lfsCfg, bool allowFormatting);
    void terminate();
    bool format();
    // Last filesystem error
    void getLastError(String *msg);
    // Last driver error
    void getDriverLastError(String *msg);

    lfs_t *getLfs() { return &lfs; }

    EEPROMLittleFsImpl();
    virtual ~EEPROMLittleFsImpl();
    virtual fs::FileImplPtr open(const char* path, const char* mode);
    virtual bool exists(const char* path);
    virtual bool rename(const char* pathFrom, const char* pathTo);
    virtual bool remove(const char* path);
    virtual bool mkdir(const char *path);
    virtual bool rmdir(const char *path);

#ifdef EEPROM_LFS_ENABLE_TESTS
//    void test(Logger *logger);
#endif
};




class EEPROMFileImpl : public fs::FileImpl {
    friend class EEPROMLittleFsImpl;
    int open(lfs_t *lfs, const char *path, const char *mode);
    int openDir(lfs_t *lfs, const char *path);
    int openFile(lfs_t *lfs, const char *path, const char *mode);
public:
    EEPROMFileImpl(EEPROMLittleFsImpl *fs);
    virtual ~EEPROMFileImpl();
    virtual size_t write(const uint8_t *buf, size_t size);
    virtual size_t read(uint8_t* buf, size_t size);
    virtual void flush();
    virtual bool seek(uint32_t pos, SeekMode mode);
    virtual size_t position() const;
    virtual size_t size() const;
    virtual void close();
    virtual time_t getLastWrite();
    virtual const char* name() const;
    virtual boolean isDirectory(void);
    virtual fs::FileImplPtr openNextFile(const char* mode);
    virtual void rewindDirectory(void);
    virtual operator bool();

private:
    EEPROMLittleFsImpl *fs;
    lfs_t *lfs;
    String fileName;
    bool isDir;
    lfs_file file;
    lfs_dir_t dir;
    bool isOpen;
    int lastError;
};


#endif
#endif