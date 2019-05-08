#!/usr/bin/env python
import glob, os
import datetime
import sys

ledseq_header = open("led_seq.h" ,"w")
ledseq_source = open("led_seq.c" ,"w")

#TODO read the heading in the sequence to create the appropriate
#enums (RED, GREEN etc.), defines and mention so in the header too
header1 = """/**
 * @file led_seq.h Header to access the PWM values for the
 *                 one or more color LED(s) for the different sequences 
 *
 * Automagically created on: """
date = datetime.datetime.now().strftime('%d-%m-%Y at %H:%M')

source1 = """/**
 * @file led_seq.c Contains the PWM values for the one or more color LED(s)
 *                 for the different sequences 
 *
 * Automagically created on: """

source2 = """ */

#include "led_seq.h"
#include "boards.h"

"""

source3 = """
const uint32_t led_pin_num[LED_COLOR_MAX] = {LED_RED, LED_GREEN};

const uint32_t * const led_seq_get_pin_ptr(void)
{
    return led_pin_num;
}

uint32_t led_seq_get_pin_num(led_sequences seq)
{
    return led_num_len[seq];
}

uint32_t led_seq_get_seg_len(led_sequences seq)
{
    return led_seq_len[seq];
}

uint16_t * led_seq_get_seq_color_ptr(led_sequences seq, led_seq_color color)
{
    return ((uint16_t *)(led_seq_ptr[seq] + color*led_seq_len[seq]));
}

uint16_t * led_seq_get_seq_duration_ptr(led_sequences seq)
{
    return (uint16_t *) led_seq_duration_ptr[seq];
}

"""

header2 = "\n */\n\n#ifndef _LED_SEQ_H_\n#define _LED_SEQ_H_\n\n#include <stdint.h>\n"
rgb_strut = """#define LED_STRUCT1(name, count)       const struct {            \\
                                        uint16_t red[count];    \\
                                      } __attribute__((packed)) name

#define LED_STRUCT2(name, count)       const struct {            \\
                                        uint16_t red[count];    \\
                                        uint16_t green[count];   \\
                                      } __attribute__((packed)) name
"""

enum_def = """
/** Specify the different LED patterns possible */
typedef enum {\n"""

enum_end = """  LED_SEQ_NULL = 255,
} led_sequences;

/** @brief The different color LEDs present in this instance */
typedef enum {
  LED_COLOR_RED,
  LED_COLOR_GREEN,
  /// To specify the number of LEDs presents
  LED_COLOR_MAX
}led_seq_color;

/**
 * @brief Gets the pointer to the array containing LED pin numbers
 *  with a length of @ref LED_COLOR_MAX
 * @return Constant pointer to a const array for reason mention in brief
 */
const uint32_t * const led_seq_get_pin_ptr(void);

/**
 * @brief Gets the number of LEDs used in the sequence
 * @param seq The sequence to query the number of LEDs
 * @return Returns the number of LEDs in the sequence
 */
uint32_t led_seq_get_pin_num(led_sequences seq);

/**
 * @brief Get the number of segments in the sequence
 * @param seq The sequence whose number of segments is required
 * @return The number of segments in the parameter's sequence
 */
uint32_t led_seq_get_seg_len(led_sequences seq);

/**
 * @brief Gets the pointer to the start of values for a color for a sequence
 * @param seq The sequence whose pointer is required
 * @param color The color in question
 * @return Pointer to the start of a color in a sequence
 */
uint16_t * led_seq_get_seq_color_ptr(led_sequences seq, led_seq_color color);

/**
 * @brief Gets the pointer to the start of segment duration array for a sequence
 * @param seq The sequence whose pointer is required
 * @return Pointer to the start of duration in a sequence
 */
uint16_t * led_seq_get_seq_duration_ptr(led_sequences seq);

#endif /* _LED_SEQ_H_ */
"""

unit_time_ms = 1

pwm_max_value = 1000
inp_res = 1000

ledseq_header.write(header1 + date + header2)

ledseq_source.write(source1 + date + source2 + rgb_strut)

ledseq_header.write(enum_def)

i = 0
for file in glob.glob("*.txt"):
	curr_seq = file.split(".")[0]
	ledseq_header.write("  LED_SEQ_" + curr_seq.upper() + " = " + str(i) + ",\n" )
	i=i+1

ledseq_header.write(enum_end)

len_list = []
seq_list = []
duration_list = []
led_num_list = []

for file in glob.glob("*.txt"):
	ip0_file = open(file, "r")
	ip0_txt = ip0_file.read().replace("\r","\n")
	ip0 = ip0_txt.split("\n")
	ip0_file.close()

	curr_seq = file.split(".")[0]
	len_list.append(len(ip0))
	seq_list.append("(uint16_t *) &" + str(curr_seq) + "_seq")
	duration_list.append(str(curr_seq) + "_duration")

	num_elements = 0

	R_list = []
	G_list = []
	ms_list = []

	for line in ip0:
		line = line.replace(";","")
		num_elements = len(line.split())
		if num_elements == 3 :
			R, G, ms = line.split()
			R = round((int(R) * pwm_max_value/inp_res),0)
			G = round((int(G) * pwm_max_value/inp_res),0)
			ms = round((int(ms)/unit_time_ms),0)
			R_list.append(int(R))
			G_list.append(int(G))
			ms_list.append(int(ms))
		elif num_elements == 2 :
			R, ms = line.split()
			R = round((int(R) * pwm_max_value/inp_res),0)			
			ms = round((int(ms)/unit_time_ms),0)
			R_list.append(int(R))
			ms_list.append(int(ms))
		else:
			sys.exit("Error: "+ file + "has number of elements that's not 2 or 3")

	if num_elements == 3 :
		ledseq_source.write("\nLED_STRUCT2(" + curr_seq + "_seq, " + str(len(ip0)) +") = {\n")
		ledseq_source.write("    {" + str(R_list).strip('[]') + "},\n")
		ledseq_source.write("    {" + str(G_list).strip('[]') + "}\n};\n")

	if num_elements == 2 :
		ledseq_source.write("\nLED_STRUCT1(" + curr_seq + "_seq, " + str(len(ip0)) +") = {\n")
		ledseq_source.write("    {" + str(R_list).strip('[]') + "}\n};\n")

	total_duration = sum(ms_list) - ms_list[0]
	ms_list.append(total_duration)
	led_num_list.append(num_elements-1)

	ledseq_source.write("const uint16_t " + curr_seq + "_duration[] = { " + str(ms_list).strip('[]') + " };\n")

ledseq_source.write("\nconst uint32_t led_num_len[] = {" + str(led_num_list).strip('[]') + "};\n")
ledseq_source.write("const uint32_t led_seq_len[] = {" + str(len_list).strip('[]') + "};\n")
ledseq_source.write("const uint16_t * const led_seq_ptr[] = {" + ', '.join(seq_list) + " };\n")
ledseq_source.write("const uint16_t * const led_seq_duration_ptr[] = { " + ', '.join(duration_list) + " };\n")

#ledseq_header.write(header3)
ledseq_source.write(source3)

ledseq_header.close()
ledseq_source.close()
