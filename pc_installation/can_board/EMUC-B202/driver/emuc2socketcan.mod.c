#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xf704969, "module_layout" },
	{ 0xb23734c0, "register_netdevice" },
	{ 0x4a165127, "kobject_put" },
	{ 0x2f2c95c4, "flush_work" },
	{ 0x2d3385d3, "system_wq" },
	{ 0xd731cdd9, "kmalloc_caches" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0x3854774b, "kstrtoll" },
	{ 0xc7a4fbed, "rtnl_lock" },
	{ 0xc3690fc, "_raw_spin_lock_bh" },
	{ 0xe870bf1, "tty_devnum" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x4c55f5ef, "pv_ops" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x8a1d3e3e, "tty_unregister_ldisc" },
	{ 0xe2d5255a, "strcmp" },
	{ 0x6fabae87, "kobject_create_and_add" },
	{ 0x408bdc34, "__netdev_alloc_skb" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x5a694d03, "netif_rx_ni" },
	{ 0x9898d6aa, "netif_tx_wake_queue" },
	{ 0xfc8b7ff6, "free_netdev" },
	{ 0x9166fada, "strncpy" },
	{ 0x5945d51c, "dev_close" },
	{ 0xdbd16536, "kfree_skb_reason" },
	{ 0x882a1d87, "alloc_candev_mqs" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0xc6cbbc89, "capable" },
	{ 0xc865cafc, "unregister_candev" },
	{ 0xa916b694, "strnlen" },
	{ 0xe46021ca, "_raw_spin_unlock_bh" },
	{ 0xd0da656b, "__stack_chk_fail" },
	{ 0x92997ed8, "_printk" },
	{ 0x65487097, "__x86_indirect_thunk_rax" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0xcc5005fe, "msleep_interruptible" },
	{ 0x7c797b6, "kmem_cache_alloc_trace" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0xf5fee30c, "tty_hangup" },
	{ 0xc3055d20, "usleep_range_state" },
	{ 0x8d5dbe75, "kernel_kobj" },
	{ 0x37a0cba, "kfree" },
	{ 0x69acdf38, "memcpy" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0xb7f1310b, "tty_mode_ioctl" },
	{ 0xf3ce59aa, "sysfs_create_file_ns" },
	{ 0x45b734d8, "tty_register_ldisc" },
	{ 0x5a1f8d90, "skb_put" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0x6e720ff2, "rtnl_unlock" },
	{ 0x9e7d6bd0, "__udelay" },
	{ 0x88db9f48, "__check_object_size" },
};

MODULE_INFO(depends, "can-dev");


MODULE_INFO(srcversion, "C911336DD216DCE1A2B8BAA");
