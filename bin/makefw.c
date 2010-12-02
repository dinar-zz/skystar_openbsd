/* makes include file from firmware files */
#include <stdio.h>
#include <fcntl.h>

void 
print_data(char *data, int fl)
{
	int             i, j;
	char           *p = data;
	for (i = fl / 16; i > 1; i--) {
		for (j = 0; j < 16; j++)
			printf("0x%02x,", ((int) (*p++)) & 0xFF);
		printf("\n");
	}
	j = fl & 0xF;
	if (!j)
		j = 16;
	for (; j > 1; j--)
		printf("0x%02x,", ((int) (*p++)) & 0xFF);
	printf("0x%02x\n", ((int) (*p)) & 0xFF);
}

int 
main(int argc, char **argv)
{
	int             fh;
	int             fl;
	char           *data;

	fh = open("Boot_up.axf", O_RDONLY);
	if (fh < 0) {
		fprintf(stderr, "Unable to open Boot_up.axf\n");
		return 1;
	}
	fl = lseek(fh, 0, SEEK_END);
	lseek(fh, 0, SEEK_SET);
	data = (char *) malloc(fl);
	read(fh, data, fl);
	close(fh);
	printf("unsigned long fw_bootup_len = %d;\n", fl);
	printf("unsigned char fw_bootup[%d] = {\n", fl);
	print_data(data, fl);
	printf("};\n\n");
	free(data);

	fh = open("Root", O_RDONLY);
	if (fh < 0) {
		fprintf(stderr, "Unable to open Root\n");
		return 1;
	}
	fl = lseek(fh, 0, SEEK_END);
	lseek(fh, 0, SEEK_SET);
	data = (char *) malloc(fl);
	read(fh, data, fl);
	close(fh);
	printf("unsigned long fw_root_len = %d;\n", fl);
	printf("unsigned char fw_root[%d] = {\n", fl);
	print_data(data, fl);
	printf("};\n\n");
	free(data);
	fh = open("Dpram", O_RDONLY);
	if (fh < 0) {
		fprintf(stderr, "Unable to open Dpram\n");
		return 1;
	}
	fl = lseek(fh, 0, SEEK_END);
	lseek(fh, 0, SEEK_SET);
	data = (char *) malloc(fl);
	read(fh, data, fl);
	close(fh);
	printf("unsigned long fw_dpram_len = %d;\n", fl);
	printf("unsigned char fw_dpram[%d] = {\n", fl);
	print_data(data, fl);
	printf("};\n\n");
	free(data);
	return 0;
}
