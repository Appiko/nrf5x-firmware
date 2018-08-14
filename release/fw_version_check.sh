#!/bin/bash
#
extract_ver_int()
{
    ver_no=""
    major_no=""
    minor_no=""
    patch_no=""
    j=0
    sp_char_pos=""

    for ((i=0 ; i<${#1}; i++))
    do
    {
        if [ "${1:$i:1}" = "$2" ]
        then
        {
            sp_char_pos[j++]=$i
            
        }
        fi
    }
    done

    major_no=${1:0:${sp_char_pos[0]}}
    if [ "$2" = "." ]
    then
        major_no=`expr $major_no \* 10000`
    fi

    diff_minor=`expr ${sp_char_pos[1]} - ${sp_char_pos[0]} - 1`
    minor_no=${1:`expr ${sp_char_pos[0]} + 1`: $diff_minor}
    if [ "$2" = "." ]
    then
        minor_no=`expr $minor_no \* 100`
    fi

    diff_patch=`expr ${#1} - ${sp_char_pos[1]} - 1`
    patch_no=${1:`expr ${sp_char_pos[1]} + 1`: $diff_patch}

    if [ "$2" = "." ]
    then
        ver_no=`expr $major_no + $minor_no + $patch_no`
    elif [ "$2" = "_" ]
    then
        ver_no="$(echo $major_no' '$minor_no' '$patch_no)"
    fi

    echo $ver_no
}

sd_ver_int=""
fw_ver_int=""
bl_ver="1.0.0"

pwd="$(pwd | tr '/' ' ' | awk '{print $NF}')"
echo $pwd

fw_ver="$(git tag --list "SensePi_"[00-99]"."[00-99]"."[00-99] | sort | tail -n1 )"
echo "FW_VER = "$fw_ver
fw_ver="$(echo $fw_ver | tr '_' ' ' | awk '{print $NF}')"
fw_ver_int=$( extract_ver_int $fw_ver "." )
echo "FW_VER_INT = "$fw_ver_int

hw_ver="$(awk '/BOARD /{print $3}' Makefile | tr '_' ' ' )"
echo "HW_VER = "$hw_ver
hw_ver_minor="$(echo $hw_ver | awk '{print $NF}')"
hw_ver_minor="$(echo ${hw_ver_minor//[A-Z]/})"
hw_ver_major="$(echo $hw_ver | awk '{print $2}')"
hw_ver_major="$(awk -v var1="$hw_ver_major" '$1 ~ var1 {print $2}' ../../release/boards_lookup)"
hw_ver_major=` expr $hw_ver_major \* 100 `
hw_ver_int=`expr $hw_ver_major + $hw_ver_minor`
echo "HW_VER_INT = "$hw_ver_int

sd_used="$(awk '/SD_USED /{print $3}' Makefile)"
echo "SD_USED = "$sd_used
sd_ver="$(awk '/SD_VER /{print $3}' Makefile)"
echo "SD_VER = "$sd_ver
sd_id="$(awk -v name_var="$sd_used" -v ver_var="$sd_ver" '$1 ~ name_var && $2 ~ ver_var {print $3}' ../../release/sd_lookup)"
echo "SD_ID = "$sd_id

bl_hex_name="$(awk -v bl_used="$bl_ver" '$1 ~ bl_used {print $2}' ../../release/bootloader_lookup)"

bl_ver_int=$( extract_ver_int $bl_ver "." )
echo "BL_VER_INT = "$bl_ver_int

make_release="$(make FW_VER_VAL=$fw_ver_int LOGGER=LOG_NONE clean_all)"

echo $make_release

ls ../../release/${pwd} || mkdir ../../release/${pwd}

nrfutil_settings_gen="$(nrfutil settings generate --family NRF52810 --application ./build/$pwd.hex --application-version $fw_ver_int --application-version-string "$fw_ver" --bootloader-version $bl_ver_int --bl-settings-version 1  ../../release/${pwd}/${pwd}_bl_settings.hex)"

nrfutil_pkg_gen="$(nrfutil pkg generate --application build/$pwd.hex --application-version $fw_ver_int --application-version-string "$fw_ver" --hw-version $hw_ver_int --sd-req "$sd_id" --key-file ../../../dfu_nrf_sdk15/examples/dfu/key_file.pem ../../release/${pwd}/${pwd}_${fw_ver_int}_010.zip)"

out_hex="$(srec_cat build/${pwd}_${sd_used}.hex --Intel ../../../dfu_nrf_sdk15/examples/dfu/secure_bootloader/pca10040e_ble/armgcc/_build/$bl_hex_name.hex --Intel ../../release/${pwd}/${pwd}_bl_settings.hex --Intel -O ../../release/${pwd}/${pwd}_${fw_ver_int}_output.hex --Intel)"

(rm ../../release/${pwd}/${pwd}_bl_settings.hex)
