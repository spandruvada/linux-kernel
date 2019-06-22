// SPDX-License-Identifier: GPL-2.0
/*
 * Intel dynamic_speed_select -- Test stub to test via simulation
 * Copyright (c) 2019 Intel Corporation.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static FILE *fp_read;
static FILE *fp_write;

int isst_read_reg(unsigned short reg, unsigned int *val)
{
	char str[80];
	char reg_str[80];
	int line = 0;

	fp_read = fopen("test-rd.dat", "rw+");

	if (!fp_read)
		return -1;

	fseek(fp_read, 0, SEEK_SET);
	snprintf(reg_str, sizeof(reg_str), "0x%x", reg);

	while (!feof(fp_read)) {
		if (!fgets(str, sizeof(str), fp_read))
			continue;
		if (!feof(fp_read)) {
			if (strstr(str, reg_str)) {
				char *val_str;
				int ret;

				val_str = strstr(str, "::");
				val_str += 2;
				ret = sscanf(val_str, "%x", val);
				if (ret == EOF)
					return -1;

				fclose(fp_read);
				return 0;
			}
		}
		++line;
	}

	fclose(fp_read);

	return -1;
}

int isst_write_reg(int reg, unsigned int val)
{
	char str[80];
	char reg_str[80];
	int line = 0;
	int pos;

	fp_write = fopen("test-wr.dat", "rw+");
	if (!fp_write)
		return -1;

	if (fseek(fp_write, 0, SEEK_SET)) {
		fclose(fp_write);
		return -1;
	}

	snprintf(reg_str, sizeof(reg_str), "0x%x", reg);

	while (!feof(fp_write)) {
		pos = ftell(fp_write);
		if (pos == -1)
			return -1;

		if (!fgets(str, sizeof(str), fp_write))
			continue;

		if (!feof(fp_write)) {
			if (strstr(str, reg_str)) {
				char *val_str;

				val_str = strstr(str, "::");
				val_str += 2;
				sprintf(val_str, "0x%08x\n", val);
				if (fseek(fp_write, pos, SEEK_SET))
					break;
				fputs(str, fp_write);
				fclose(fp_write);
				return 0;
			}
		}
		++line;
	}

	fclose(fp_write);

	return 0;
}
