#include <CompilationOpts.h>

#ifdef USE_EEPROM_LFS
#include <lfs.h>
#include <HardwareSerial.h>
#include <LogMgr.h>
#include "EEPROMLittleFs.h"
#include "EEPROMDriver.h"

//#define DEBUG_PRINT(s) s;
#define DEBUG_PRINT(s)

// Read a region in a block. Negative error codes are propogated
// to the user.
int eepromLfsRead(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    EEPROMLittleFsImpl *elfs = (EEPROMLittleFsImpl *)c->context;
#ifdef EEPROM_LFS_IN_MEMORY
    memcpy(buffer, elfs->diskSpace + block * c->block_size + off, size);
    return 0;
#else
    bool rc = elfs->eeDriver.read(block * c->block_size + off, (uint8_t *)buffer, size);
if (elfs->eeDriver.lastError() != 0) {
    DEBUG_PRINT(Serial.printf("   EEPROM Driver error: %s\n", elfs->eeDriver.lastErrorStr()))
}
    return (rc ? 0 : LFS_ERR_IO);
#endif
}

// Program a region in a block. The block must have previously
// been erased. Negative error codes are prospogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int eepromLfsProg(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    EEPROMLittleFsImpl *elfs = (EEPROMLittleFsImpl *)c->context;
#ifdef EEPROM_LFS_IN_MEMORY
    memcpy(elfs->diskSpace + block * c->block_size + off, buffer, size);
    return 0;
#else
    bool rc = elfs->eeDriver.write(block * c->block_size + off, (uint8_t *)buffer, size);
if (elfs->eeDriver.lastError() != 0) {
    DEBUG_PRINT(Serial.printf("   EEPROM Driver error: %s\n", elfs->eeDriver.lastErrorStr()))
}
    return (rc ? 0 : LFS_ERR_IO);
#endif
}

// Erase a block. A block must be erased before being programmed.
// The state of an erased block is undefined. Negative error codes
// are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int eepromLfsErase(const struct lfs_config *c, lfs_block_t block)
{
    return 0;
}

// Sync the state of the underlying block device. Negative error codes
// are propogated to the user.
int eepromLfsSync(const struct lfs_config *c)
{
    EEPROMLittleFsImpl *elfs = (EEPROMLittleFsImpl *)c->context;
#ifdef EEPROM_LFS_IN_MEMORY
    return 0;
#else
    bool rc = elfs->eeDriver.sync();
    return (rc ? 0 : LFS_ERR_IO);
#endif
}



EEPROMLittleFsImpl::EEPROMLittleFsImpl()
{
    lastError = 0;
    isInitialized = false;
}

EEPROMLittleFsImpl::~EEPROMLittleFsImpl()
{
    terminate();
}

bool EEPROMLittleFsImpl::init(int i2cPort, const EEPROMLittleFs::EEPROMConfig &eeCfg,
    const EEPROMLittleFs::LFSConfig &lfsCfg, bool allowFormatting)
{
    isInitialized = false;

    cfg = { };
    cfg.context = this;

    // block device operations
    cfg.read  = eepromLfsRead;
    cfg.prog  = eepromLfsProg;
    cfg.erase = eepromLfsErase;
    cfg.sync  = eepromLfsSync;

    // block device configuration
    cfg.read_size = 1;
    cfg.prog_size = 1;
    cfg.block_size = lfsCfg.block_size;
    cfg.block_count = eeCfg.size / cfg.block_size;
    cfg.block_cycles = lfsCfg.block_cycles;
    cfg.cache_size = lfsCfg.cache_size;
    cfg.lookahead_size = lfsCfg.lookahead_size;

#ifdef EEPROM_LFS_IN_MEMORY
    diskSpace = (uint8_t*)malloc(cfg.block_size * cfg.block_count);
    if (diskSpace == nullptr) {
        lastError = LFS_ERR_NOMEM;
        isInitialized = false;
        return false;
    }
    memset(diskSpace, 0, cfg.block_size * cfg.block_count);
#else
    bool rc = eeDriver.init(i2cPort, eeCfg.deviceAddress);
    if (!rc) {
        DEBUG_PRINT(Serial.printf("EEPROM Driver initialization failure: %s", eeDriver.lastErrorStr()))
        lastError = LFS_ERR_IO;
        return false;
    }
#endif

    lastError = mount(allowFormatting);
    isInitialized = (lastError == 0);
    return isInitialized;
}

bool EEPROMLittleFsImpl::init(const EEPROMLittleFs::I2CConfig &i2cCfg, const EEPROMLittleFs::EEPROMConfig &eeCfg,
    const EEPROMLittleFs::LFSConfig &lfsCfg, bool allowFormatting)
{
    isInitialized = false;

    cfg = { };
    cfg.context = this;

    // block device operations
    cfg.read  = eepromLfsRead;
    cfg.prog  = eepromLfsProg;
    cfg.erase = eepromLfsErase;
    cfg.sync  = eepromLfsSync;

    // block device configuration
    cfg.read_size = 1;
    cfg.prog_size = 1;
    cfg.block_size = lfsCfg.block_size;
    cfg.block_count = eeCfg.size / cfg.block_size;
    cfg.block_cycles = lfsCfg.block_cycles;
    cfg.cache_size = lfsCfg.cache_size;
    cfg.lookahead_size = lfsCfg.lookahead_size;

#ifdef EEPROM_LFS_IN_MEMORY
    diskSpace = (uint8_t*)malloc(cfg.block_size * cfg.block_count);
    if (diskSpace == nullptr) {
        lastError = LFS_ERR_NOMEM;
        isInitialized = false;
        return false;
    }
    memset(diskSpace, 0, cfg.block_size * cfg.block_count);
#else
    bool rc = eeDriver.init(i2cCfg.sdaPin, i2cCfg.sclPin, i2cCfg.freqHz, i2cCfg.port, eeCfg.deviceAddress);
    if (!rc) {
        DEBUG_PRINT(Serial.printf("EEPROM Driver initialization failure: %s", eeDriver.lastErrorStr()))
        lastError = LFS_ERR_IO;
        return false;
    }
#endif

    lastError = mount(allowFormatting);
    isInitialized = (lastError == 0);
    return isInitialized;
}

int EEPROMLittleFsImpl::mount(bool allowFormatting)
{
    int err = lfs_mount(&lfs, &cfg);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err < 0 && allowFormatting) {
        DEBUG_PRINT(Serial.println("Formatting..."))
        err = lfs_format(&lfs, &cfg);
        DEBUG_PRINT(Serial.printf("Formatted, error %d, mounting...\n", err))
        err = lfs_mount(&lfs, &cfg);
        DEBUG_PRINT(Serial.printf("Mounted, error %d\n", err))
    }
    return err;
}

void EEPROMLittleFsImpl::terminate()
{
    if (isInitialized) {
        lfs_unmount(&lfs);
        isInitialized = false;
    }
}

bool EEPROMLittleFsImpl::format()
{
    bool mustRemount = false;
    if (isInitialized) {
        lfs_unmount(&lfs);
        mustRemount = true;
    }

    lastError = lfs_format(&lfs, &cfg);
    if (lastError < 0) {
        return false;
    }
    if (mustRemount) {
        lastError = lfs_mount(&lfs, &cfg);
    }
    return lastError;
}

const char *EEPROMLittleFsImpl::errorStr(int err)
{
    switch (err) {
        case LFS_ERR_OK: return "LFS_ERR_OK: No error";
        case LFS_ERR_IO: return "LFS_ERR_IO: Error during device operation";
        case LFS_ERR_CORRUPT: return "LFS_ERR_CORRUPT: Corrupted";
        case LFS_ERR_NOENT: return "LFS_ERR_NOENT: No directory entry";
        case LFS_ERR_EXIST: return "LFS_ERR_EXIST: Entry already exists";
        case LFS_ERR_NOTDIR: return "LFS_ERR_NOTDIR: Entry is not a dir";
        case LFS_ERR_ISDIR: return "LFS_ERR_ISDIR: Entry is a dir";
        case LFS_ERR_NOTEMPTY: return "LFS_ERR_NOTEMPTY: Dir is not empty";
        case LFS_ERR_BADF: return "LFS_ERR_BADF: Bad file number";
        case LFS_ERR_FBIG: return "LFS_ERR_FBIG: File too large";
        case LFS_ERR_INVAL: return "LFS_ERR_INVAL: Invalid parameter";
        case LFS_ERR_NOSPC: return "LFS_ERR_NOSPC: No space left on device";
        case LFS_ERR_NOMEM: return "LFS_ERR_NOMEM: No more memory available";
        case LFS_ERR_NOATTR: return "LFS_ERR_NOATTR: No data/attr available";
        case LFS_ERR_NAMETOOLONG: return "LFS_ERR_NAMETOOLONG: File name too long";
        default: return "Unknown";
    }
};

fs::FileImplPtr EEPROMLittleFsImpl::open(const char* path, const char* mode)
{
    if (!isInitialized) {
        return nullptr;
    }

    std::shared_ptr<EEPROMFileImpl> f = std::make_shared<EEPROMFileImpl>(this);
    int rc = f->open(&lfs, path, mode);
    if (rc < 0) {
        lastError = rc;
        return nullptr;
    } else {
        return f;
    }
}

bool EEPROMLittleFsImpl::exists(const char* path)
{
    if (!isInitialized) {
        return false;
    }
    lfs_info info;
    int rc = lfs_stat(&lfs, path, &info);
    lastError = rc;
    return !(rc < 0);
}

bool EEPROMLittleFsImpl::rename(const char* pathFrom, const char* pathTo)
{
    if (!isInitialized) {
        return false;
    }
    int rc = lfs_rename(&lfs, pathFrom, pathTo);
    lastError = rc;
    return !(rc < 0);
}

bool EEPROMLittleFsImpl::remove(const char* path)
{
    if (!isInitialized) {
        return false;
    }
    int rc = lfs_remove(&lfs, path);
    lastError = rc;
    return !(rc < 0);
}

bool EEPROMLittleFsImpl::mkdir(const char *path)
{
    if (!isInitialized) {
        return false;
    }
    int rc = lfs_mkdir(&lfs, path);
    lastError = rc;
    return !(rc < 0);
}

bool EEPROMLittleFsImpl::rmdir(const char *path)
{
    if (!isInitialized) {
        return false;
    }
    int rc = lfs_remove(&lfs, path);
    lastError = rc;
    return !(rc < 0);
}

void EEPROMLittleFsImpl::getLastError(String *msg)
{
    msg->concat(errorStr(lastError));
}

void EEPROMLittleFsImpl::getDriverLastError(String *msg)
{
    msg->concat(eeDriver.lastErrorStr());
}


EEPROMLittleFs::EEPROMLittleFs()
: fs::FS(std::make_shared<EEPROMLittleFsImpl>())
{
    this->fsImplPtr = (EEPROMLittleFsImpl *)this->_impl.get();
}

bool EEPROMLittleFs::init(const I2CConfig &i2cCfg, const EEPROMConfig &eeCfg, const LFSConfig &lfsCfg, bool allowFormatting)
{
    return fsImplPtr->init(i2cCfg, eeCfg, lfsCfg, allowFormatting);
}

bool EEPROMLittleFs::init(int i2cPort, const EEPROMConfig &eeCfg, const LFSConfig &lfsCfg, bool allowFormatting)
{
    return fsImplPtr->init(i2cPort, eeCfg, lfsCfg, allowFormatting);
}

void EEPROMLittleFs::terminate()
{
    fsImplPtr->terminate();
}

bool EEPROMLittleFs::format()
{
    return fsImplPtr->format();
}

void EEPROMLittleFs::getLastError(String *msg)
{
    fsImplPtr->getLastError(msg);
}

void EEPROMLittleFs::getDriverLastError(String *msg)
{
    fsImplPtr->getDriverLastError(msg);
}



EEPROMFileImpl::EEPROMFileImpl(EEPROMLittleFsImpl *fs)
{
    this->fs = fs;
    this->lfs = fs->getLfs();
    lastError = LFS_ERR_INVAL;
    isOpen = false;
    isDir = false;
}

EEPROMFileImpl::~EEPROMFileImpl()
{
    if (isOpen) {
        close();
    }
}

int EEPROMFileImpl::open(lfs_t *lfs, const char *path, const char *mode)
{
    DEBUG_PRINT(Serial.printf("In open(%s, %s)\n", path, mode))
    this->lfs = lfs;

    lfs_info info;
    lastError = lfs_stat(lfs, path, &info);
    if (lastError == LFS_ERR_NOENT) {
        return openFile(lfs, path, mode);
    }
    if (lastError < 0) {
        return lastError;
    }

    if (info.type == LFS_TYPE_DIR) {
        return openDir(lfs, path);
    } else {
        return openFile(lfs, path, mode);
    }
}

int EEPROMFileImpl::openDir(lfs_t *lfs, const char *path)
{
    isDir = true;
    fileName = path;
    lastError = lfs_dir_open(lfs, &dir, path);
    isOpen = !(lastError < 0);
    DEBUG_PRINT(Serial.printf("In open(%s,<>), type: DIR, rc = %d\n", path, lastError))
    return lastError;
}

int EEPROMFileImpl::openFile(lfs_t *lfs, const char *path, const char *mode)
{
    isDir = false;
    fileName = path;
    int flags = 0;
    if (strcmp(mode, "r") == 0) {
        flags = LFS_O_RDONLY;
    } else if (strcmp(mode, "w") == 0) {
        flags = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC;
    } else if (strcmp(mode, "a") == 0) {
        flags = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND;
    } else if (strcmp(mode, "r+") == 0) {
        flags = LFS_O_RDWR;
    } else if (strcmp(mode, "w+") == 0) {
        flags = LFS_O_RDWR | LFS_O_CREAT | LFS_O_TRUNC;
    } else if (strcmp(mode, "a+") == 0) {
        flags = LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND;
    } else {
        return LFS_ERR_INVAL;
    }
    lastError = lfs_file_open(lfs, &file, path, flags);
    isOpen = !(lastError < 0);
    DEBUG_PRINT(Serial.printf("In open(%s, %s), type: FILE, rc = %d\n", path, mode, lastError))
    return lastError;
}

size_t EEPROMFileImpl::write(const uint8_t *buf, size_t size)
{
    DEBUG_PRINT(Serial.printf("In write(<>, %d) %s\n", size, fileName.c_str()))
    if (isDir) {
        lastError = LFS_ERR_ISDIR;
        return 0;
    }
    if (!isOpen) {
        lastError = LFS_ERR_INVAL;
        return 0;
    }

    int32_t rc = lfs_file_write(lfs, &file, buf, size);
    lastError = (rc < 0 ? rc : 0);
    DEBUG_PRINT(Serial.printf("In write(<>, %d) rc=%d %s\n", size, rc, fileName.c_str()))
    return rc < 0 ? 0 : rc;
}

size_t EEPROMFileImpl::read(uint8_t* buf, size_t size)
{
    DEBUG_PRINT(Serial.printf("In read(<>, %d) %s\n", size, fileName.c_str()))
    if (isDir) {
        lastError = LFS_ERR_ISDIR;
        return 0;
    }
    if (!isOpen) {
        lastError = LFS_ERR_INVAL;
        return 0;
    }
    int32_t rc = lfs_file_read(lfs, &file, buf, size);
    lastError = (rc < 0 ? rc : 0);
    DEBUG_PRINT(Serial.printf("In read(<>, %d) rc=%d %s\n", size, rc, fileName.c_str()))
    return rc < 0 ? 0 : rc;
}

void EEPROMFileImpl::flush()
{
    DEBUG_PRINT(Serial.printf("In flush() %s\n", fileName.c_str()))
    if (isDir) {
        lastError = LFS_ERR_ISDIR;
        return;
    }
    if (!isOpen) {
        lastError = LFS_ERR_INVAL;
        return;
    }
    lastError = lfs_file_sync(lfs, &file);
    DEBUG_PRINT(Serial.printf("In flush() lastError=%d %s\n", lastError, fileName.c_str()))
}

bool EEPROMFileImpl::seek(uint32_t pos, SeekMode mode)
{
    DEBUG_PRINT(Serial.printf("In seek(%d, %d) %s\n", pos, mode, fileName.c_str()))
    if (isDir) {
        lastError = LFS_ERR_ISDIR;
        return false;
    }
    if (!isOpen) {
        lastError = LFS_ERR_INVAL;
        return false;
    }

    int rc = lfs_file_seek(lfs, &file, pos, mode);
    lastError = (rc < 0 ? rc : 0);
    DEBUG_PRINT(Serial.printf("In seek(%d, %d) rc=%d %s\n", pos, mode, rc, fileName.c_str()))
    return !(rc < 0);
}

size_t EEPROMFileImpl::position() const
{
    DEBUG_PRINT(Serial.printf("In position() %s\n", fileName.c_str()))
    if (isDir) {
        const_cast<EEPROMFileImpl*>(this)->lastError = LFS_ERR_ISDIR;
        return 0;
    }
    if (!isOpen) {
        const_cast<EEPROMFileImpl*>(this)->lastError = LFS_ERR_INVAL;
        return 0;
    }
    int32_t rc = lfs_file_tell(lfs, const_cast<lfs_file_t *>(&file));
    DEBUG_PRINT(Serial.printf("In position() rc=%d %s\n", rc, fileName.c_str()))
    return rc >= 0 ? rc : 0;
}

size_t EEPROMFileImpl::size() const
{
    DEBUG_PRINT(Serial.printf("In size() %s\n", fileName.c_str()))
    if (isDir) {
        const_cast<EEPROMFileImpl*>(this)->lastError = LFS_ERR_ISDIR;
        return 0;
    }
    if (!isOpen) {
        const_cast<EEPROMFileImpl*>(this)->lastError = LFS_ERR_INVAL;
        return 0;
    }
    int32_t rc = lfs_file_size(lfs, const_cast<lfs_file_t *>(&file));
    DEBUG_PRINT(Serial.printf("In size() rc=%d %s\n", rc, fileName.c_str()))
    return rc >= 0 ? rc : 0;
}

void EEPROMFileImpl::close()
{
    DEBUG_PRINT(Serial.printf("In close() %s\n", fileName.c_str()))
    if (!isOpen) {
        DEBUG_PRINT(Serial.printf("In close() returning because not open %s\n", fileName.c_str()))
        lastError = LFS_ERR_INVAL;
        return;
    }
    if (isDir) {
        DEBUG_PRINT(Serial.printf("Will call lfs_dir_close() %s\n", fileName.c_str()))
        lastError = lfs_dir_close(lfs, &dir);
    } else {
        DEBUG_PRINT(Serial.printf("Will call lfs_file_close() %s\n", fileName.c_str()))
        lastError = lfs_file_close(lfs, &file);
    }
    isOpen = false;
    DEBUG_PRINT(Serial.printf("In close() lastError=%d\n", lastError))
}

time_t EEPROMFileImpl::getLastWrite()
{
    return 0;
}

const char *EEPROMFileImpl::name() const
{
    return fileName.c_str();
}

boolean EEPROMFileImpl::isDirectory(void)
{
    return isDir;
}

fs::FileImplPtr EEPROMFileImpl::openNextFile(const char* mode)
{
    DEBUG_PRINT(Serial.printf("In openNextFile(%s) %s\n", mode, fileName.c_str()))
    if (!isDir) {
        lastError = LFS_ERR_NOTDIR;
        return std::make_shared<EEPROMFileImpl>(fs); // not initialized
    }
    if (!isOpen) {
        lastError = LFS_ERR_INVAL;
        return std::make_shared<EEPROMFileImpl>(fs); // not initialized
    }
    lfs_info info;
    lastError = lfs_dir_read(lfs, &dir, &info);
    if (lastError < 0 || lastError == 0) { // 0: end of directory
        DEBUG_PRINT(Serial.printf("In openNextFile() (after lfs_dir_read()) lastError=%d %s\n", lastError, fileName.c_str()))
        return std::make_shared<EEPROMFileImpl>(fs); // not initialized
    }
    std::shared_ptr<EEPROMFileImpl> newFile = std::make_shared<EEPROMFileImpl>(fs);
    if (info.type == LFS_TYPE_DIR) {
        lastError = newFile->openDir(lfs, info.name);
    } else {
        lastError = newFile->openFile(lfs, info.name, mode);
    }
    DEBUG_PRINT(Serial.printf("In openNextFile() lastError=%d %s\n", lastError, fileName.c_str()))

    if (lastError < 0) {
        return std::make_shared<EEPROMFileImpl>(fs); // not initialized
    }
    return newFile;
}

void EEPROMFileImpl::rewindDirectory(void)
{
    if (!isDir) {
        lastError = LFS_ERR_NOTDIR;
        return;
    }
    if (!isOpen) {
        lastError = LFS_ERR_INVAL;
        return;
    }
    lastError = lfs_dir_rewind(lfs, &dir);
}

EEPROMFileImpl::operator bool()
{
    return lastError == 0;
}

#endif
