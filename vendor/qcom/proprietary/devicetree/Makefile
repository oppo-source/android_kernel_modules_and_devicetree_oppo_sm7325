vendor := $(srctree)/$(src)

#ifndef VENDOR_EDIT
#ifneq "$(wildcard $(vendor)/qcom)" ""
#	subdir-y += qcom
#endif
#else
#	subdir-y += 20031
	subdir-y += qcom
ifeq ($(strip $(OPLUS_VND_BUILD_PLATFORM)), SM7325)
        subdir-y += oplus/walle
        subdir-y += oplus/lantian
        subdir-y += oplus/taylor
        subdir-y += oplus/garbo
        subdir-y += oplus/ziti
else
	subdir-y += oplus/luwu

endif
#endif
