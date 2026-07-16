#include <stdio.h> // printf
#include <wchar.h> // wchar_t

#include <hidapi.h>

#define MAX_STR 255

	int x;
	int y;
	int z;
	int pitch;
	int yaw;
	int roll;

void parseBuf(unsigned char* buf){


		if (buf[0] == 1){
			 pitch = (short)(buf[2] << 8) | buf[1];
			 yaw = (short)(buf[4] << 8) | buf[3];
			 roll = (short)(buf[6] << 8) | buf[5];
			
		} else {
			 x = (short)(buf[2] << 8) | buf[1];
			 y = (short)(buf[4] << 8) | buf[3];
			 z = (short)(buf[6] << 8) | buf[5];
		}
	
}

int bufSize = 10;
int main(int argc, char* argv[])
{
	int res;
	unsigned char buf[bufSize];
	wchar_t wstr[MAX_STR];
	hid_device *handle;
	int i;

	// Initialize the hidapi library
	res = hid_init();

	// Open the device using the VID, PID,
	// and optionally the Serial number.
	handle = hid_open(0x256F , 0xc635, NULL);
	if (!handle) {
		printf("Unable to open device\n");
		hid_exit();
 		return 1;
	}

	

	hid_set_nonblocking(handle,1);

	while (true){
		res = hid_read(handle, buf, bufSize);
		res = hid_read(handle, buf, bufSize);
		parseBuf(buf);
		printf("%i %i %i %i %i %i\n",x,y,z,pitch,yaw,roll);
		printf("\n");
	}
	
	// for (int i = 0; i<bufSize; i++){

	// 			printf("%i ",buf[i]);
			
	// 		}
	
	// Read requested state
	//res = hid_read(handle, buf, bufSize);

	// Close the device
	//hid_close(handle);

	// Finalize the hidapi library
	res = hid_exit();

	return 0;
}