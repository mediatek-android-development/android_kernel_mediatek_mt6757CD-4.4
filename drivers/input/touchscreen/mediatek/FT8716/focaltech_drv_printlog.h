//lichenggan and for mstar IC to print key log
//#define TPD_DEVICE_TPLINK_90X "FTS_8XXX"

#define TPLINK_TPD_DEVICE           "tplink-tpd"

#define TPD_ERR(a, arg...)          do{ pr_err(TPLINK_TPD_DEVICE ": " a, ##arg); }while(0)

#ifdef TPD_DEBUG
#undef TPD_DEBUG                    /*override TPD_DEBUG in tpd.h*/
#endif
#define TPD_DEBUG(a, arg...)        do{ pr_debug(TPLINK_TPD_DEVICE ": " a, ##arg); }while(0)

#define TPD_ENABLE_INFO
#ifdef TPD_ENABLE_INFO
//#define TPD_INFO(fmt, args...)      do{ printk(TPLINK_TPD_DEVICE ": " fmt, ## args); }while(0)
#define TPD_INFO(a, args...)        do{ pr_info(TPLINK_TPD_DEVICE ": " a, ##args); }while(0)
#else
#define TPD_INFO(a, args...)        do{}while(0)
#endif
