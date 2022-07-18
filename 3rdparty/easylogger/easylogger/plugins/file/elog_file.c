/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2015-2019, Qintl, <qintl_linux@163.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Save log to file.
 * Created on: 2019-01-05
 */

 #define LOG_TAG    "elog.file"

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>           
#include <unistd.h>
#include <time.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <libgen.h>
#include "elog_file.h"

/* initialize OK flag */
static bool init_ok = false;
static FILE *fp = NULL;
static ElogFileCfg local_cfg;

static uint8_t log_file_name[1024] = {0};
static ElogFileCfg log_elog_cfg;
static ElogFileCfg *log_elog_cfg_ptr = NULL;
//static char log_elog_file_name[1024] = {0};
void elog_file_setopts(char *name, size_t max_size, int max_rotate)
{
    if(name)
    {
        snprintf(log_file_name, sizeof(log_file_name), "%s%s", name, strrchr(name, '.')?"":ELOG_LOG_FILE_WITH_DATE_DEFAULT_EXTNAME);
        log_elog_cfg.name = log_file_name;
        log_elog_cfg.max_size = max_size;
        log_elog_cfg.max_rotate = max_rotate;
        log_elog_cfg_ptr = &log_elog_cfg;
        printf("elog_file_setopts name is ok\n");
        printf("elog_file is %s\n", name);
    }else
    {
        printf("elog_file_setopts name is null\n");
        return ;
    }
}

ElogErrCode elog_file_init(void)
{
    ElogErrCode result = ELOG_NO_ERR;
    ElogFileCfg cfg;

    if (init_ok)
        goto __exit;

    elog_file_port_init();

    if(log_elog_cfg_ptr)
    {
        printf("elog use config file opts\n");
        cfg.name = log_elog_cfg_ptr->name;
        cfg.max_size = log_elog_cfg_ptr->max_size;
        cfg.max_rotate = log_elog_cfg_ptr->max_rotate;
    }else
    {
        printf("elog use default file opts\n");
        cfg.name = ELOG_FILE_NAME;
        cfg.max_size = ELOG_FILE_MAX_SIZE;
        cfg.max_rotate = ELOG_FILE_MAX_ROTATE;
    }
    uint8_t file_name[1024] = {0};
    strncpy(file_name, cfg.name, sizeof(file_name));
    dirname(file_name);
    if (access(file_name, 0))
    {
        uint8_t temp[1080] = {0};
        snprintf(temp, sizeof(temp), "mkdir -p %s", file_name);
        system(temp);
    }
    
    elog_file_config(&cfg);

    init_ok = true;
__exit:
    return result;
}

/*
 * rotate the log file xxx.log.n-1 => xxx.log.n, and xxx.log => xxx.log.0
 */
static bool elog_file_rotate(void)
{
#define SUFFIX_LEN                     64
    /* mv xxx.log.n-1 => xxx.log.n, and xxx.log => xxx.log.0 */
    int n, err = 0;
    char oldpath[512] = {0}, newpath[512] = {0};
    size_t base = strlen(local_cfg.name);
    bool result = true;
    FILE *tmp_fp;

    memcpy(oldpath, local_cfg.name, base);
    memcpy(newpath, local_cfg.name, base);

#if 1 == ELOG_LOG_FILE_WITH_DATE_ENABLE
    char tempbuf[512] = {0}, starttime[64] = {0}, endtime[64] = {0};
    char *path = dirname(newpath);
    DIR *dir = NULL;
    struct dirent *ptr;
    uint64_t filetime = 0;
    uint64_t count = 0;
    char *extname = strrchr(local_cfg.name, '.');
    if (NULL == extname)
    {
        extname = "";
    }
foreachdir:
    if (path && (dir = opendir(path)))
    {
        count = 0;
        filetime = ~0;
        memset(oldpath, 0, sizeof(oldpath));
        while ((ptr=readdir(dir)) != NULL)
        {
            if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)
            {
                continue;
            }
            else if(ptr->d_type != DT_DIR)
            {  
                snprintf(tempbuf, sizeof(tempbuf), "%s/%s", path, ptr->d_name);
                if (strncmp(tempbuf, local_cfg.name, strlen(local_cfg.name)))
                {
                    int32_t offset = strlen(local_cfg.name) - strlen(extname);
                    if (offset < 0)
                    {
                        continue;
                    }
                    strncpy(tempbuf + offset, extname, strlen(extname));
                    tempbuf[offset + strlen(extname)] = 0;     
                }                   
                if (strncmp(tempbuf, local_cfg.name, strlen(local_cfg.name)))
                {
                    continue;
                }
                else
                {
                    struct stat s;
                    snprintf(tempbuf, sizeof(tempbuf), "%s/%s", path, ptr->d_name);
                    stat(tempbuf, &s);                   
                    if (filetime > (s.st_ctim.tv_sec*1000+s.st_ctim.tv_nsec/1000000))
                    {
                        filetime = s.st_ctim.tv_sec*1000+s.st_ctim.tv_nsec/1000000;
                        snprintf(oldpath, sizeof(oldpath), "%s/%s", path, ptr->d_name);
                    }
                    count++;                                                                          
                }
            }
        }
    }
    closedir(dir);
    if (count > local_cfg.max_rotate)
    {
        remove(oldpath);
        if (count > (local_cfg.max_rotate + 1) && strlen(oldpath) > 0)
        {
            goto foreachdir;
        }
    }

    fseek(fp, 0L, SEEK_SET);
    fread(starttime, sizeof(starttime) - 1, 1, fp);
    fclose(fp);
    if (strstr(starttime, ELOG_LOG_FILE_WITH_DATE_MAGIC_NUMBER))
    {
        strstr(starttime, ELOG_LOG_FILE_WITH_DATE_MAGIC_NUMBER)[0] = 0;
    }
    else
    {
        snprintf(starttime, sizeof(starttime), ELOG_LOG_FILE_DATE_FORMAT, 1970,1,1,1,1,1);
    }
    
    time_t tt = time(NULL);
    struct tm s_t, *t = localtime_r(&tt, &s_t);
	snprintf(endtime, sizeof(endtime), ELOG_LOG_FILE_DATE_FORMAT, (t->tm_year + 1900),t->tm_mon + 1,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);
    memset(tempbuf, 0, sizeof(tempbuf));
    memcpy(tempbuf, local_cfg.name, base);
    snprintf(tempbuf + base - strlen(extname), SUFFIX_LEN - strlen(extname), "_%s_%s%s", starttime, endtime, extname);
    rename(local_cfg.name, tempbuf);
    remove(local_cfg.name);
#else

    fclose(fp);

    for (n = local_cfg.max_rotate - 1; n >= 0; --n) {
        snprintf(oldpath + base, SUFFIX_LEN, n ? ".%d" : "", n - 1);
        snprintf(newpath + base, SUFFIX_LEN, ".%d", n);
        /* remove the old file */
        if ((tmp_fp = fopen(newpath , "r")) != NULL) {
            fclose(tmp_fp);
            remove(newpath);
        }
        /* change the new log file to old file name */
        if ((tmp_fp = fopen(oldpath , "r")) != NULL) {
            fclose(tmp_fp);
            err = rename(oldpath, newpath);
        }

        if (err < 0) {
            result = false;
            goto __exit;
        }
    }

__exit:

#endif
    /* reopen the file */
    fp = fopen(local_cfg.name, "a+");
    #if 1 == ELOG_LOG_FILE_WITH_DATE_ENABLE
        fwrite(endtime, strlen(endtime), 1, fp);
        fwrite(ELOG_LOG_FILE_WITH_DATE_MAGIC_NUMBER, strlen(ELOG_LOG_FILE_WITH_DATE_MAGIC_NUMBER), 1, fp);
        fflush(fp);
    #endif
    return result;
}


void elog_file_write(const char *log, size_t size)
{
    size_t file_size = 0;

    ELOG_ASSERT(init_ok);
    ELOG_ASSERT(log);

    elog_file_port_lock();

    fseek(fp, 0L, SEEK_END);
    file_size = ftell(fp);

    if (unlikely(file_size > local_cfg.max_size)) {
#if ELOG_FILE_MAX_ROTATE > 0
        if (!elog_file_rotate()) {
            goto __exit;
        }
#else
        goto __exit;
#endif
    }

    fwrite(log, size, 1, fp);

#ifdef ELOG_FILE_FLUSH_CAHCE_ENABLE
    fflush(fp);
#endif

__exit:
    elog_file_port_unlock();
}

void elog_file_deinit(void)
{
    ELOG_ASSERT(init_ok);

    ElogFileCfg cfg = {NULL, 0, 0};

    elog_file_config(&cfg);

    elog_file_port_deinit();

    init_ok = false;
}

void elog_file_config(ElogFileCfg *cfg)
{
    elog_file_port_lock();

    if (fp) {
        fclose(fp);
        fp = NULL;
    }

    if (cfg != NULL) {
        local_cfg.name = cfg->name;
        local_cfg.max_size = cfg->max_size;
        local_cfg.max_rotate = cfg->max_rotate;

        if (local_cfg.name != NULL && strlen(local_cfg.name) > 0)
            fp = fopen(local_cfg.name, "a+");
        #if 1 == ELOG_LOG_FILE_WITH_DATE_ENABLE          
            fseek(fp, 0L, SEEK_END);
            if (ftell(fp) == 0)
            {
                char timestamp[32] = {0};
                time_t tt = time(NULL);
                struct tm s_t;
                struct tm *t = localtime_r(&tt, &s_t);	
                sprintf(timestamp, ELOG_LOG_FILE_DATE_FORMAT, (t->tm_year + 1900),t->tm_mon + 1,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);
                fwrite(timestamp, strlen(timestamp), 1, fp);
                fwrite(ELOG_LOG_FILE_WITH_DATE_MAGIC_NUMBER, strlen(ELOG_LOG_FILE_WITH_DATE_MAGIC_NUMBER), 1, fp);
                fflush(fp);
            }
        #endif    
    }

    elog_file_port_unlock();
}
