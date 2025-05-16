/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>
#include <math.h>
#include <zephyr/data/json.h>
#include <zephyr/sys/rb.h>
#include <string.h>
#include <zephyr/drivers/display.h>
#include <lvgl.h>

#define JSON_BUFFER_SIZE 100

K_SEM_DEFINE(screenSem, 0, 1);
K_MSGQ_DEFINE(kalmanXQueue, sizeof(double), 1, 1);
K_MSGQ_DEFINE(kalmanYQueue, sizeof(double), 1, 1);
K_MSGQ_DEFINE(rssiXQueue, sizeof(double), 1, 1);
K_MSGQ_DEFINE(rssiYQueue, sizeof(double), 1, 1);

// Sensor object for use with JSON descriptor
struct tagioObj {
    char* variable;
	char* value;
    char* unit;
};

// JSON object descriptor for generating JSON messages
static const struct json_obj_descr tagioObjDescriptor[] = {
    JSON_OBJ_DESCR_PRIM(struct tagioObj, variable, JSON_TOK_STRING),
    JSON_OBJ_DESCR_PRIM(struct tagioObj, value, JSON_TOK_STRING),
    JSON_OBJ_DESCR_PRIM(struct tagioObj, unit,  JSON_TOK_STRING)
};

char jsonDataBuf[JSON_BUFFER_SIZE] = {0};

 struct ibeacon_node {
    struct rbnode rb_n;
    char name[32];
	char mac[18];
	uint16_t major;
	uint16_t minor;
	float x;
	float y;
	char left[32];
	char right[32];
 };

 static bool ibeacon_less_than();

 struct rbtree ibeacon_tree = { 
	.lessthan_fn = ibeacon_less_than 
};

typedef struct KalmanFilter {
    float measuredErr;
    float estimatedErr;
    float noise;
    float currEstimate;
    float lastEstimate;
    float kalmanGain;
}KalmanFilter;

 // constants for calculating distance from rssi
 #define CALIBRATED_RSSI	-55.f
 #define ENVIRONMENTAL_CONSTANT 3.f

 // Calibrated rssi at 1m for every node (GRID A)
 #define CALIBRATED_RSSIS	{-45.f, -58.f, -55.f, -53.f, -47.f, -59.f, -62.f, -47.f, -63.f, -60.f, -67.f, -69.f, -59.f}

 // number of nodes for localisation
 #define N 13
 // number of dimensions for localisation
 #define DIM 2

 // hard coded grid	TODO

 char nodeChar[13] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M'};

 #define DIST_MAX_ENTRIES	20

 K_MSGQ_DEFINE(distQ, sizeof(double*), 1, 1);
 K_MSGQ_DEFINE(coordQ, sizeof(float**), 1, 1); 
 K_MSGQ_DEFINE(gridQ, sizeof(char), 1, 1); 
 K_MSGQ_DEFINE(ultrasonicQ, sizeof(float*), 1, 1); 

 // temp variable to store joined ultrasonic node transmission as uint32
 uint32_t ultrasonicDist_u32 = 0;

 // current rssi readings
 int8_t rssiArr[13] = {0};

 #define FUCKYOU {	{0.0f, 0.0f}, {1.5f, 0.0f}, {3.0f, 0.0f}, {3.0f, 2.0f}, {3.0f, 4.0f}, {1.5f, 4.0f}, {0.0f, 4.0f}, {0.0f, 2.0f}, {4.5f, 0.0f}, {6.0f, 0.0f}, {6.0f, 2.0f}, {6.0f, 4.0f}, {4.5f, 4.0f}	}
 #define IHATEYOU {	{ 0,  1}, { 0,  1}, { 0,  1}, {-1,  0}, { 0, -1}, { 0, -1}, { 0, -1}, { 1,  0}, { 0,  1}, { 0,  1}, {-1,  0}, { 0, -1}, { 0, -1} }

 // array storing the mobile distance from each node in stations array
 double distances[N] = {0};

 static struct ibeacon_node *node_to_remove = NULL;
 static struct ibeacon_node *node_to_print = NULL;

 // function prototype for scanning
 static void start_scan(void);

 // uart shell device pointer
 const struct device* dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));

 static bool ibeacon_less_than(struct rbnode *a, struct rbnode *b) {

	 struct ibeacon_node *node_a = CONTAINER_OF(a, struct ibeacon_node, rb_n);
	 struct ibeacon_node *node_b = CONTAINER_OF(b, struct ibeacon_node, rb_n);
	 return strcmp(node_a->name, node_b->name) < 0;
 }

 int add_ibeacon_node(const char *name, const char *mac, 
	uint16_t major, uint16_t minor, 
	float x, float y, const char *left, 
	const char *right) {

    struct ibeacon_node *new_node = k_malloc(sizeof(struct ibeacon_node));
    if (!new_node) {
        printk("Failed to allocate memory for new iBeacon node\n");
        return -ENOMEM;
    }

    strncpy(new_node->name, name, sizeof(new_node->name) - 1);
    new_node->name[sizeof(new_node->name) - 1] = '\0';
    strncpy(new_node->mac, mac, sizeof(new_node->mac) - 1);
    new_node->mac[sizeof(new_node->mac) - 1] = '\0';
    new_node->major = major;
    new_node->minor = minor;
    new_node->x = x;
    new_node->y = y;
    strncpy(new_node->left, left, sizeof(new_node->left) - 1);
    new_node->left[sizeof(new_node->left) - 1] = '\0';
    strncpy(new_node->right, right, sizeof(new_node->right) - 1);
    new_node->right[sizeof(new_node->right) - 1] = '\0';

    rb_insert(&ibeacon_tree, &new_node->rb_n);
    printk("iBeacon node '%s' added.\n", new_node->name);

	float** coordPointer;
	k_msgq_peek(&coordQ, &coordPointer);
	float compareCoord[13][2] = FUCKYOU;

	for (int i = 0; i < 13; i++) {
		if (compareCoord[i][0] == x) {
			if (compareCoord[i][1] == y) {
				//CHANGED
				float* coord = (float*)k_calloc(2, sizeof(float));
				coord[0] = x;
				coord[1] = y;
				coordPointer[i] = coord;
			}
		}
	}
    return 0;
}

static void find_ibeacon_node_for_removal_callback(struct rbnode *node, void *cookie) {
    const char *search_name = (const char *)cookie;
    struct ibeacon_node *ibeacon = CONTAINER_OF(node, struct ibeacon_node, rb_n);

    if (strcmp(ibeacon->name, search_name) == 0) {
        node_to_remove = ibeacon;

		float** coordPointer;
		k_msgq_peek(&coordQ, &coordPointer);
		float compareCoord[13][2] = FUCKYOU;
	
		for (int i = 0; i < 13; i++) {
			if (compareCoord[i][0] == node_to_remove->x) {
				if (compareCoord[i][1] == node_to_remove->y) {
					k_free(coordPointer[i]);
					coordPointer[i] = NULL;
				}
			}
		}
        return;
    }
    return;
}

int remove_ibeacon_node(struct rbtree *tree, const char *name) {
    node_to_remove = NULL;
    rb_walk(tree, find_ibeacon_node_for_removal_callback, (void *)name);

    if (node_to_remove) {
        rb_remove(tree, &node_to_remove->rb_n);
        printk("iBeacon node '%s' removed.\n", node_to_remove->name);
        k_free(node_to_remove);
        node_to_remove = NULL;
        return 0;
    } else {
        printk("iBeacon node with MAC '%s' not found.\n", name);
        return -ENOENT;
    }
}

static void find_ibeacon_node_callback(struct rbnode *node, void *cookie) {
    const char *search_name = (const char *)cookie;
    struct ibeacon_node *ibeacon = CONTAINER_OF(node, struct ibeacon_node, rb_n);

    if (strcmp(ibeacon->name, search_name) == 0) {
        // i found you .-.
        //printk("Found iBeacon node: %s (%s)\n", ibeacon->name, ibeacon->mac);
		node_to_print = ibeacon;
        return;
    }

    return;
}

// Function to initiate the search
struct ibeacon_node *find_ibeacon_node(struct rbtree *tree, const char *name) {
	node_to_print = NULL;
    rb_walk(tree, find_ibeacon_node_callback, (void *)name);

    return node_to_print;
}

int view_ibeacon_node(const char *name) {
    struct ibeacon_node *found_node = find_ibeacon_node(&ibeacon_tree, name);

    if (found_node) {
        printk("BLE Name: %s\n", found_node->name);
        printk("BLE MAC Address: %s\n", found_node->mac);
        printk("Major: %u\n", found_node->major);
        printk("Minor: %u\n", found_node->minor);
        printk("X Coordinate: %f\n", found_node->x);
        printk("Y Coordinate: %f\n", found_node->y);
        printk("Left Neighbor: %s\n", found_node->left);
        printk("Right Neighbor: %s\n", found_node->right);
        return 0;
    } else {
        printk("iBeacon node with name '%s' not found.\n", name);
        return -ENOENT;
    }
}

static void print_ibeacon(struct rbnode *node, void *arg) {
    struct ibeacon_node *ibeacon = CONTAINER_OF(node, struct ibeacon_node, rb_n);
    printk("--------------------\n");
    printk("BLE Name: %s\n", ibeacon->name);
    printk("BLE MAC Address: %s\n", ibeacon->mac);
    printk("Major: %u\n", ibeacon->major);
    printk("Minor: %u\n", ibeacon->minor);
    printk("X Coordinate: %f\n", ibeacon->x);
    printk("Y Coordinate: %f\n", ibeacon->y);
    printk("Left Neighbor: %s\n", ibeacon->left);
    printk("Right Neighbor: %s\n", ibeacon->right);
}

int view_all_ibeacon_nodes(void) {
    printk("--- All iBeacon Nodes ---\n");
    rb_walk(&ibeacon_tree, print_ibeacon, NULL);
    printk("-------------------------\n");
    return 0;
}

 // shell command add ibeacon to grid
 int ibeacon_add(const struct shell *sh, int argc, char** argv) {
	if (argc != 9) {
        shell_print(sh, "Usage: ibeacon a <name> <mac> <major> <minor> <x> <y> <left> <right>");
        return -EINVAL;
    }

    uint16_t major = atoi(argv[3]);
    uint16_t minor = atoi(argv[4]);
    float x = atof(argv[5]);
    float y = atof(argv[6]);

    add_ibeacon_node(argv[1], argv[2], major, minor, x, y, argv[7], argv[8]);
    return 0;
 }

 // shell command remove ibeacon from grid
 int ibeacon_remove(const struct shell *sh, int argc, char** argv) {
    if (argc != 2) {
        shell_print(sh, "Usage: ibeacon r <name>");
        return -EINVAL;
    }

    remove_ibeacon_node(&ibeacon_tree, argv[1]);
    return 0;
 }

 int ibeacon_view(const struct shell *sh, int argc, char** argv) {
    if (argc == 2 && strcmp(argv[1], "-a") == 0) {
        view_all_ibeacon_nodes();
    } else if (argc == 2) {
        view_ibeacon_node(argv[1]);
    } else {
        shell_print(sh, "Usage: ibeacon v <name> or ibeacon view -a");
        return -EINVAL;
    }
    return 0;
 }

 static void find_all_ibeacon_nodes_callback(struct rbnode *node, void *cookie) {
    struct ibeacon_node *ibeacon = CONTAINER_OF(node, struct ibeacon_node, rb_n);
    struct ibeacon_node **list = (struct ibeacon_node **)cookie;

    // Store pointer in provided array
    for (int i = 0; i < 32; i++) {
        if (list[i] == NULL) {
            list[i] = ibeacon;
            break;
        }
    }
}

void remove_all_ibeacon_nodes(void) {
    struct ibeacon_node *node_list[32] = {0};  // Max 32 nodes

    rb_walk(&ibeacon_tree, find_all_ibeacon_nodes_callback, node_list);

    for (int i = 0; i < 32; i++) {
        if (node_list[i] == NULL) break;

        float **coordPointer;
        k_msgq_peek(&coordQ, &coordPointer);

        float compareCoord[13][2] = FUCKYOU;

        for (int j = 0; j < 13; j++) {
            if (compareCoord[j][0] == node_list[i]->x &&
                compareCoord[j][1] == node_list[i]->y) {
                k_free(coordPointer[j]);
                coordPointer[j] = NULL;
            }
        }

        rb_remove(&ibeacon_tree, &node_list[i]->rb_n);
        printk("iBeacon node '%s' removed.\n", node_list[i]->name);
        k_free(node_list[i]);
    }

    printk("All iBeacon nodes have been removed.\n");
}

 int ibeacon_load_grid(const struct shell *sh, int argc, char** argv) {
    if (argc != 2) {
        shell_print(sh, "Usage: ibeacon load <A|B>");
        return -EINVAL;
    }

	if (strcmp(argv[1], "A") && strcmp(argv[1], "B")) {
		return -EINVAL;
	}

    const char* grid = argv[1];
	k_msgq_purge(&gridQ);
	k_msgq_put(&gridQ, grid, K_NO_WAIT);

    // MACs for both grids
    const char* macs[][8] = {
        // Grid A
        {
            "F5:75:FE:85:34:67", "E5:73:87:06:1E:86", "CA:99:9E:FD:98:B1", "CB:1B:89:82:FF:FE",
            "D4:D2:A0:A4:5C:AC", "C1:13:27:E9:B7:7C", "F1:04:48:06:39:A0", "CA:0C:E0:DB:CE:60"
        },
        // Grid B
        {
            "CA:99:9E:FD:98:B1", "CB:1B:89:82:FF:FE", "D4:D2:A0:A4:5C:AC", "D4:7F:D4:7C:20:13",
            "F7:0B:21:F1:C8:E1", "FD:E0:8D:FA:3E:4A", "EE:32:F7:28:FA:AC", "F7:3B:46:A8:D7:2C"
        }
    };

    const char* names[][8] = {
        { "4011-A", "4011-B", "4011-C", "4011-D", "4011-E", "4011-F", "4011-G", "4011-H" },
        { "4011-C", "4011-D", "4011-E", "4011-I", "4011-J", "4011-K", "4011-L", "4011-M" }
    };

    uint16_t majors[][8] = {
        { 2753, 32975, 26679, 41747, 30679, 6195, 30525, 57395 },
        { 26679, 41747, 30679, 60345, 12249, 36748, 27564, 49247 }
    };

    uint16_t minors[][8] = {
        { 32998, 20959, 40363, 38800, 51963, 18394, 30544, 28931 },
        { 40363, 38800, 51963, 49995, 30916, 11457, 27589, 52925 }
    };

    float coords[][8][2] = {
        { {0,0},{1.5,0},{3,0},{3,2},{3,4},{1.5,4},{0,4},{0,2} }, // grid A
        { {3,0},{3,2},{3,4},{4.5,0},{6,0},{6,2},{6,4},{4.5,4} }  // grid B
    };

	const char* left[][8] = {
		{ "", "4011-A", "4011-B", "", "4011-F", "4011-G", "", "" },
        { "4011-B", "", "4011-F", "4011-C", "4011-I", "", "4011-M", "4011-E" }
	};

	
	const char* right[][8] = {
        { "4011-B", "4011-C", "4011-I", "", "4011-M", "4011-E", "4011-F", "" },
        { "4011-I", "", "4011-M", "4011-J", "", "", "", "4011-L" }
	};

    // First remove all existing nodes
    for (int i = 0; i < 13; ++i) {
        float** coordPointer;
        k_msgq_peek(&coordQ, &coordPointer);
        if (coordPointer[i] != NULL) {
            k_free(coordPointer[i]);
            coordPointer[i] = NULL;
        }
    }

	remove_all_ibeacon_nodes();

    // Select index 0 for Grid A, 1 for Grid B
    int grid_index = (strcmp(grid, "A") == 0) ? 0 : (strcmp(grid, "B") == 0) ? 1 : -1;
    if (grid_index == -1) {
        shell_print(sh, "Invalid grid. Use A or B.");
        return -EINVAL;
    }

    for (int i = 0; i < 8; ++i) {
        float x = coords[grid_index][i][0];
        float y = coords[grid_index][i][1];
        add_ibeacon_node(names[grid_index][i], macs[grid_index][i], majors[grid_index][i], minors[grid_index][i],
                         x, y, left[grid_index][i], right[grid_index][i]); // TODO actual left and right
    }

    shell_print(sh, "Grid %s loaded successfully.", grid);
    return 0;
}

 SHELL_STATIC_SUBCMD_SET_CREATE(ibeacon_cmd, 
	SHELL_CMD_ARG(a, NULL, "Adds an iBeacon node.", &ibeacon_add, 9, 0), 
	SHELL_CMD_ARG(r, NULL, "Remove an iBeacon node.", &ibeacon_remove, 2, 0), 
	SHELL_CMD_ARG(v, NULL, "View iBeacon node details.", &ibeacon_view, 2, 0), 
	SHELL_CMD_ARG(l, NULL, "Load predefined grid A or B", &ibeacon_load_grid, 2, 0),
	SHELL_SUBCMD_SET_END);
 SHELL_CMD_ARG_REGISTER(ibeacon, &ibeacon_cmd, "Add, remove or view iBeacon node details.", NULL, 2, 8);

void ultrasonic_node_set(const struct shell *sh, int argc, char** argv) {
	k_msgq_purge(&ultrasonicQ);
	int usNode = 0;
	if (!strcmp(argv[1], "A")) {
		usNode = 0;
	} else if (!strcmp(argv[1], "B")) {
		usNode = 1;

	} else if (!strcmp(argv[1], "C")) {
		usNode = 2;

	} else if (!strcmp(argv[1], "D")) {
		usNode = 3;

	} else if (!strcmp(argv[1], "E")) {
		usNode = 4;

	} else if (!strcmp(argv[1], "F")) {
		usNode = 5;

	} else if (!strcmp(argv[1], "G")) {
		usNode = 6;

	} else if (!strcmp(argv[1], "H")) {
		usNode = 7;

	} else if (!strcmp(argv[1], "I")) {
		usNode = 8;

	} else if (!strcmp(argv[1], "J")) {
		usNode = 9;

	} else if (!strcmp(argv[1], "K")) {
		usNode = 10;

	} else if (!strcmp(argv[1], "L")) {
		usNode = 11;
		
	} else if (!strcmp(argv[1], "M")) {
		usNode = 12;
	}
	k_msgq_put(&ultrasonicQ, &usNode, K_NO_WAIT);
}

 SHELL_STATIC_SUBCMD_SET_CREATE(ultrasonic_cmd,
	SHELL_CMD_ARG(s, NULL, "Sets the ultrasonic node.", &ultrasonic_node_set, 2, 0),
	SHELL_SUBCMD_SET_END);
 SHELL_CMD_ARG_REGISTER(us, &ultrasonic_cmd, "Set ultrasonic node.", NULL, 2, 0);


 // macro for converting uint32 to float while preserving bit order
 #define UINT32_TO_FLOAT(i, f) {	\
	uint32_t tempInt = i;	\	
	f = *(float*)&tempInt;	\
 }

 // helper function for joining 8 uint8s into one uint32
 uint32_t join_u32(uint8_t* fArray) {
	return (((uint32_t)fArray[0] << 24) |
			((uint32_t)fArray[1] << 16) |
			((uint32_t)fArray[2] << 8) 	|
			((uint32_t)fArray[3]) );
}
 
 // RSSI to distance calculation
 float rssi_to_dist(uint8_t nodeIndex, int8_t rssi) {
	float calibratedRssi[13] = CALIBRATED_RSSIS;
	return (powf(10.f, (calibratedRssi[nodeIndex] - rssi) / (10.f * ENVIRONMENTAL_CONSTANT)));
 }

 // return 0 if not mobile mac address
 uint8_t check_mobile(const bt_addr_le_t *addr) {
	// mac address for mobile node (THINGY52)
	uint8_t mobileAddr[6] = {0xFE, 0xC0, 0x04, 0xFD, 0x5D, 0xC5};
	for (uint8_t i = 0; i < 6; i++) {
		if (addr->a.val[i] == mobileAddr[i]) {
			if (i == 5) {
				// thats the one
				//printk("Matching iBeacon found\r\n");
				return 1;
			}
		} else {
			break;
		}
	}
	return 0;
 }

 // return 0 is not ultrasonic mac address
 uint8_t check_ultrasonic(const bt_addr_le_t *addr) {
	// mac address for ultrasonic node (NRF WITH RUSTY BUTTON) (3.0.2)
	uint8_t ultrasonicAddr[6] = {0x97, 0x6D, 0xE6, 0x30, 0x15, 0xD3};
	for (uint8_t i = 0; i < 6; i++) {
		if (addr->a.val[i] == ultrasonicAddr[i]) {
			if (i == 5) {
				// thats the one
				//printk("Matching iBeacon found\r\n");
				return 1;
			}
		} else {
			break;
		}
	}
	return 0;
 }

void multilateration_new(float** stations, double* r, double x[DIM]) {
    int valid_count = 0;
    float* valid_stations[DIST_MAX_ENTRIES];
    double valid_r[DIST_MAX_ENTRIES];

    //printk("Filtering valid stations...\n");
    for (int i = 0; i < DIST_MAX_ENTRIES; ++i) {
        if (stations[i] != NULL) {
            valid_stations[valid_count] = stations[i];
            valid_r[valid_count] = r[i];
/*             printk("  Station %d is valid: (%.2f, %.2f), distance: %.2f\n",
                   i, stations[i][0], stations[i][1], r[i]); */
            valid_count++;
        }
    }

    if (valid_count < DIM + 1) {
        printk("Not enough valid stations. Need at least %d, got %d.\n", DIM + 1, valid_count);
        return;
    }

    double A[DIST_MAX_ENTRIES][DIM];
    double b[DIST_MAX_ENTRIES];

    //printk("Building A matrix and b vector...\n");
    for (int i = 1; i < valid_count; ++i) {
        double r_diff = valid_r[0] * valid_r[0] - valid_r[i] * valid_r[i];
        double c_diff = 0.0;

        for (int d = 0; d < DIM; ++d) {
            A[i - 1][d] = 2.0 * ((double)valid_stations[i][d] - (double)valid_stations[0][d]);
            c_diff += valid_stations[i][d] * valid_stations[i][d] -
                      valid_stations[0][d] * valid_stations[0][d];
        }

        b[i - 1] = r_diff + c_diff;

        //printk("  Row %d: A = [%.2f, %.2f], b = %.2f\n", i - 1, A[i - 1][0], A[i - 1][1], b[i - 1]);
    }

    int equations = valid_count - 1;

    double AtA[DIM][DIM] = {0};
    double Atb[DIM] = {0};

    //printk("Computing AtA and Atb...\n");
	const double epsilon = 1e-6;
	for (int i = 0; i < DIM; ++i) {
		for (int j = 0; j < DIM; ++j) {
			for (int k = 0; k < equations; ++k) {
				double weight = 1.0 / (valid_r[k + 1] * valid_r[k + 1] + epsilon);  // k+1 because A starts from index 1
				AtA[i][j] += weight * A[k][i] * A[k][j];
			}
			//printk("  AtA[%d][%d] = %.4f\n", i, j, AtA[i][j]);
		}

		for (int k = 0; k < equations; ++k) {
			double weight = 1.0 / (valid_r[k + 1] * valid_r[k + 1] + epsilon);
			Atb[i] += weight * A[k][i] * b[k];
		}
	}

    double det = AtA[0][0] * AtA[1][1] - AtA[0][1] * AtA[1][0];
    
    if (fabs(det) < 1e-10) {
        printk("Matrix is singular or nearly singular.\n");
        return;
    }

    x[0] = (Atb[0] * AtA[1][1] - Atb[1] * AtA[0][1]) / det;
    x[1] = (Atb[1] * AtA[0][0] - Atb[0] * AtA[1][0]) / det;

}

float estimate(float measurement, KalmanFilter* kalman) {
    // Update Kalman Gain
    kalman->kalmanGain = kalman->estimatedErr / (kalman->estimatedErr + kalman->measuredErr);
    
    // Get the current estimate
    kalman->currEstimate = kalman->lastEstimate + kalman->kalmanGain * (measurement - kalman->lastEstimate);
    
    // Get the estimated error
    kalman->estimatedErr = (1.f - kalman->kalmanGain) * kalman->estimatedErr + fabsf(kalman->lastEstimate - kalman->currEstimate) * kalman->noise;
    // Update the estimates
    kalman->lastEstimate = kalman->currEstimate;

    return kalman->currEstimate;
}
 
// ad parsing function for mobile adv
static bool parse_mobile_cb(struct bt_data *data, void *user_data) {
	 if (data->type == BT_DATA_MANUFACTURER_DATA && data->data_len >= 25) {
		 const uint8_t *d = data->data;

		 struct tagioObj jsonObj;

         jsonObj.value = (char*)k_malloc(sizeof(char) * 5);
         jsonObj.variable = (char*)k_malloc(sizeof(char) * 10);
         jsonObj.unit = (char*)k_malloc(sizeof(char) * 4);

		 // check valid indices
		 if ((d[20] < 13)) {
			double* stupid;
			k_msgq_peek(&distQ, &stupid);

			// store rssi data
			rssiArr[d[20]] = (int8_t)d[21];
			// calculate and store distance using rssi
			stupid[d[20]] = rssi_to_dist(d[20], rssiArr[d[20]]);

			// print int  to string
            snprintf(jsonObj.value, 5, "%d", rssiArr[d[20]]);

			// Create the variable string for the JSON
            snprintf(jsonObj.variable, 10, "noderssi%d", (uint8_t)d[20]);
            
            // Set the units
            strcpy(jsonObj.unit, "dBm");
            
            // Encode the JSON
            json_obj_encode_buf(tagioObjDescriptor, 3, &jsonObj, jsonDataBuf, JSON_BUFFER_SIZE);
            
			//TODO uncommment
            //printk("%s\r\n", jsonDataBuf);
            memset(jsonDataBuf, 0, JSON_BUFFER_SIZE);

			if (d[22] < 13) {
				// store rssi data
				rssiArr[d[22]] = (int8_t)d[23];
				// calculate and store distance using rssi
				stupid[d[22]] = rssi_to_dist(d[22], rssiArr[d[22]]);

				// print int to a string
				snprintf(jsonObj.value, 5, "%d", rssiArr[d[22]]);

				// Create the variable string for the JSON
				snprintf(jsonObj.variable, 10, "noderssi%d", (uint8_t)d[22]);
	
				// Set the units
				strcpy(jsonObj.unit, "dBm");
				
				// Encode the JSON
				json_obj_encode_buf(tagioObjDescriptor, 3, &jsonObj, jsonDataBuf, JSON_BUFFER_SIZE);
				
				// TODO uncomment this
				//printk("%s\r\n", jsonDataBuf);
				memset(jsonDataBuf, 0, JSON_BUFFER_SIZE);
			}	
		 } else {
			// invalid index
			printk("Index out of range\r\n");
		 }
		 k_free(jsonObj.variable);
		 k_free(jsonObj.unit);
		 k_free(jsonObj.value);

	 }
	 return true;
 }

 // ad parsing function for ultrasonic adv
 static bool parse_us_cb(struct bt_data *data, void *user_data) {
	// temp variable to store individual 4 bytes from ultrasonic node transmission
	uint8_t float_part[4] = {0};
	if (data->type == BT_DATA_MANUFACTURER_DATA && data->data_len >= 25) {
		const uint8_t *d = data->data;
		// store the 4 bytes making up uint32
		for (int i = 0; i < 4; i++) {
			float_part[i] = d[20 + i];
		}
		// join the 4 bytes to make uint32
		ultrasonicDist_u32 = join_u32(float_part);
	}
	return true;
}
 
// callback function for when device found
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad) { 
	 // only check indirect scannable ads
	 if (type != BT_GAP_ADV_TYPE_ADV_SCAN_IND) {
		// check scannable not connectable
		 return;
	 }
	 // check if mac matches mobile
	 if (check_mobile(addr)) {
		//printk("MOBILE FOUND\r\n");
		bt_data_parse(ad, parse_mobile_cb, NULL);
		return;
	 }
	 // check if mac matches ultrasonic
	 if (check_ultrasonic(addr)) {
		//printk("ULTRASONIC FOUND\r\n");
		bt_data_parse(ad, parse_us_cb, NULL);
		return;
	 }

}
 
 // function that sets up bt scanning
 static void start_scan(void) {
	 int err;
 
	 err = bt_le_scan_start(BT_LE_SCAN_PASSIVE_CONTINUOUS, device_found);
	 if (err) {
		 printk("Scanning failed to start (err %d)\n", err);
		 return;
	 }
 
	 printk("Scanning successfully started\n");
 }

 // helper function that rounds float to nearest 0.5
 float round_to_half(float x) {
	return round(x * 2.0) / 2.0;
 }

lv_obj_t* draw_circle_outline() {

    static lv_obj_t* circle;
    circle = lv_obj_create(lv_scr_act());
    lv_obj_set_scrollbar_mode(circle, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_size(circle, 8, 8);
    lv_obj_set_pos(circle, 64-6, 32-6);
    lv_obj_set_style_bg_color(circle, (lv_color_t)LV_COLOR_MAKE(255, 255, 255), 0);
    lv_obj_set_style_radius(circle, LV_RADIUS_CIRCLE, 0);

    // Set border color and width
    lv_obj_set_style_border_color(circle, (lv_color_t)LV_COLOR_MAKE(0,0,0), LV_PART_MAIN);
    lv_obj_set_style_border_width(circle, 1, LV_PART_MAIN);
    
    return circle;
}

 /* Draw Rectangle
  * 
  * Draws a rectangle at a given (x,y)
  *
  */
 lv_obj_t* draw_rect(int x, int y)
 {
    static lv_obj_t* node;
    node = lv_obj_create(lv_scr_act());
    lv_obj_set_size(node, 8, 8);
    lv_obj_set_pos(node, x, y);
    lv_obj_set_style_bg_color(node, (lv_color_t)LV_COLOR_MAKE(0,0,0), 0);
    return node;
 }

void analysis_thread()
{
    printk("in analysis thread\r\n");
    float ultrasonicKal;
	int coordDir[13][2] = IHATEYOU;
	float compareCoord[13][2] = FUCKYOU;

	int usNode;
	char grid;
	// DISTANCE ARRAY
	double* stupid = (double*)k_calloc(DIST_MAX_ENTRIES, sizeof(double));
	k_msgq_put(&distQ, &stupid, K_NO_WAIT);

	// COORD ARRAY
	float** retarded = (float**)k_calloc(DIST_MAX_ENTRIES, sizeof(float*));
	k_msgq_put(&coordQ, &retarded, K_NO_WAIT);

	 // array to store estimated coordinates
	 double estimated[DIM];

	// var to store ultrasonic distance
	float usDistance = 0;

	// prev tick for prev pos
	int32_t prevTime = 0;

	// var to store previous pos
	float lastPos[2] = {0};

	// var to store cumulative distance travelled
	float cumDist = 0;

	// temp var for putting current position
	float tempDiff[2] = {0};
	float tempDist = 0;

	float currV = 0;

	int32_t prevValidReadingTime = 0;
	float yPos = 0;
	float xPos = 0;
	float rssiXValues[3] = {0};
	float rssiYValues[3] = {0};
	uint8_t rssiCount = 0;

    // ultrasonic and beacon data
    KalmanFilter ultrasonicKalman = { 
        .kalmanGain = 0.f,
        .lastEstimate = 0.f,
        .currEstimate = 0.f,
        .measuredErr = 0.012f,
        .estimatedErr = 1.f,
        .noise = 0.1f
    };

	KalmanFilter yKalman = { 
		.kalmanGain = 0.f,
		.lastEstimate = 0.f,
		.currEstimate = 0.f,
		.measuredErr = 0.5f,
		.estimatedErr = 1.f,
		.noise = 0.1f
	};

	KalmanFilter xKalman = { 
		.kalmanGain = 0.f,
		.lastEstimate = 0.f,
		.currEstimate = 0.f,
		.measuredErr = 0.5f,
		.estimatedErr = 1.f,
		.noise = 0.1f
	};

	struct tagioObj fuck;
     // var for rssi moving window average
	 uint8_t rssiIndex = 0;

  	 while (1) {
		
        // Get the estimated position data via multilateration
		multilateration_new(retarded, stupid, estimated);

		k_msgq_peek(&gridQ, &grid);

		if (grid == 'B') {
			estimated[0] -= 3;
		}

		if (estimated[0] > 3) {
			estimated[0] = 3;
		} else if (estimated[0] < 0) {
			estimated[0] = 0;
		}
		if (estimated[1] > 4) {
			estimated[1] = 4;
		} else if (estimated[1] < 0) {
			estimated[1] = 0;
		}

		rssiXValues[rssiIndex] = estimated[0];
		rssiYValues[rssiIndex] = estimated[1];

		rssiIndex = (rssiIndex + 1) % 3;

		// Calculate average
		float sumX = 0, sumY = 0;
		for (int i = 0; i < 3; i++) {
			sumX += rssiXValues[i];
			sumY += rssiYValues[i];
		}

		estimated[0] = sumX / 3;
		estimated[1] = sumY / 3;
		printk("Averaged RSSI Position: (%f, %f)\r\n", estimated[0], estimated[1]);
        double rssiX = estimated[0];
        double rssiY = estimated[1];

		// Use Kalman filter to estimate the X Position
		xPos = estimate(estimated[0], &xKalman);
		yPos = estimate(estimated[1], &yKalman);

        // put that shit in the queue
        k_msgq_put(&kalmanXQueue, &xPos, K_NO_WAIT);
        k_msgq_put(&kalmanYQueue, &yPos, K_NO_WAIT);

        k_msgq_put(&rssiXQueue, &rssiX, K_NO_WAIT);
        k_msgq_put(&rssiYQueue, &rssiY, K_NO_WAIT);
	
        // Ultrasonic node distance
		UINT32_TO_FLOAT(ultrasonicDist_u32, usDistance);

		// Check if there is a valid reading for the ultrasonic node
		if (usDistance < 10.f) {
			// 300ms between a valid reading ensures we don't have
			// some bullshit data
			if (k_uptime_get_32() - prevValidReadingTime < 1500) {
				// Fuse the 2 datasets through average
				//double avgYPos = (estimated[1] + (stations[1][1] + distance)) / 2;
				//printk("Averaged Position: (%f, %f)\r\n", estimated[0], avgYPos);
				
				// Estimate ultrasonic distance
				//double ultrasonicKal = estimate(usDistance, &ultrasonicKalman);

				// Average the kalman data, therefore fusing them
				k_msgq_peek(&ultrasonicQ, &usNode);

				float baseX = compareCoord[usNode][0];
				float baseY = compareCoord[usNode][1];
				float dirX = coordDir[usNode][0];
				float dirY = coordDir[usNode][1];

				float ultrasonicX = baseX + dirX * usDistance;
				float ultrasonicY = baseY + dirY * usDistance;

				float ultrasonicCoord;

				// If the node faces along the Y-axis
				if (dirY) {
					printk("IN DIR Y\r\n");
					ultrasonicCoord = baseY + dirY * usDistance;
					printk("Pre Average:	%f\r\n", ultrasonicCoord);
					ultrasonicKal = estimate(ultrasonicCoord, &ultrasonicKalman);
					yPos = (ultrasonicKal + yPos) / 2;
					printk("Post Average:	%f\r\n", yPos);
				}
				// If the node faces along the X-axis
				else if (dirX) {
					printk("IN DIR X\r\n");
					ultrasonicCoord = baseX + dirX * usDistance;
					printk("Pre Average:	%f\r\n", ultrasonicCoord);
					float xUltrasonicKal = estimate(ultrasonicCoord, &ultrasonicKalman);
					xPos = (xUltrasonicKal + xPos) / 2;
					printk("Post Average:	%f\r\n", xPos);

				}		
			} 
			// Update the new valid reading time
			prevValidReadingTime = k_uptime_get_32();
        }
		
		//printk("Distance: %fm\r\n", distance);
        //float estimatedPosition = estimate(distance, &kalman);
        printk("Kalman Position: (%f, %f)\r\n", xPos, yPos); 

		// if first iteration
		if (prevTime == 0) {
			// set last position
			lastPos[0] = xPos;
			lastPos[1] = yPos;
			prevTime = k_uptime_get_32();
		} else {

			// delta x
			tempDiff[0] = xPos - lastPos[0];
			
            // delta y
			tempDiff[1] = yPos - lastPos[1];
			
            // distance from last position
			tempDist = sqrt((tempDiff[0] * tempDiff[0]) + (tempDiff[1] * tempDiff[1]));
			
            // add to cumulative dist travelled
			cumDist += tempDist;
			
            // find current velocity
			currV = tempDist / ((k_uptime_get_32() - prevTime) / 1000.0f);
			
            // update previous time, last position
			prevTime = k_uptime_get_32();
			lastPos[0] = xPos;
			lastPos[1] = yPos;


			fuck.value = (char*)k_malloc(sizeof(char) * 15);
			fuck.variable = (char*)k_malloc(sizeof(char) * 10);
			fuck.unit = (char*)k_malloc(sizeof(char) * 4);

			// print int  to string
			//printk("Cumulative Distance:	%.3f\r\n", cumDist);
            snprintf(fuck.value, 10, "%.3f", cumDist);

			// Create the variable string for the JSON
            snprintf(fuck.variable, 10, "distance");
            
            // Set the units
            strcpy(fuck.unit, "m");
            
            // Encode the JSON
            json_obj_encode_buf(tagioObjDescriptor, 3, &fuck, jsonDataBuf, JSON_BUFFER_SIZE);
            
			// TODO uncomment this
            //printk("%s\r\n", jsonDataBuf);
            memset(jsonDataBuf, 0, JSON_BUFFER_SIZE);

			// print int  to string
			//printk("Velocity:	%.3f\r\n", currV);
			snprintf(fuck.value, 10, "%.3f", currV);

			// Create the variable string for the JSON
			snprintf(fuck.variable, 10, "velocity");
			
			// Set the units
			strcpy(fuck.unit, "m/s");
			
			// Encode the JSON
			json_obj_encode_buf(tagioObjDescriptor, 3, &fuck, jsonDataBuf, JSON_BUFFER_SIZE);
			
			// TODO uncomment this
			//printk("%s\r\n", jsonDataBuf);
			memset(jsonDataBuf, 0, JSON_BUFFER_SIZE);

			k_free(fuck.variable);
			k_free(fuck.unit);
			k_free(fuck.value);
		}
        k_sem_give(&screenSem);
        k_msleep(500);
    }
}

void comms_thread()
{
    int err;

    printk("In comms thread");
    // attempt to enable bt
    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }
    printk("Bluetooth initialized\n");

    // start scanning
    start_scan();
    return;
}

void screen_thread()
{
    printk("in screen thread\r\n");
    static lv_obj_t* node1;
    node1 = draw_rect(33, 0);
    
    static lv_obj_t* node2;
    node2 = draw_rect(60, 0);

    static lv_obj_t* node3;
    node3 = draw_rect(87, 0);

    static lv_obj_t* node4;
    node4 = draw_rect(33, 28);
    
    static lv_obj_t* node5;
    node5 = draw_rect(87, 28);
    
    static lv_obj_t* node6;
    node6 = draw_rect(33, 55); 
    
    static lv_obj_t* node7;
    node7 = draw_rect(60, 55);
    
    static lv_obj_t* node8;
    node8 = draw_rect(87, 55); 
 
    static lv_obj_t* circle;
    circle = lv_obj_create(lv_scr_act());
    lv_obj_set_scrollbar_mode(circle, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_size(circle, 12, 12);
    lv_obj_set_pos(circle, 64-6, 32-6);
    lv_obj_set_style_bg_color(circle, (lv_color_t)LV_COLOR_MAKE(0, 0, 0), 0);
    lv_obj_set_style_radius(circle, LV_RADIUS_CIRCLE, 0);

    static lv_obj_t* rssiCircle;
    rssiCircle = draw_circle_outline();
    double xPos = 0;
    double yPos = 0;
    double rssiXPos = 0;
    double rssiYPos = 0;

    while (1) {
        k_sem_take(&screenSem, K_FOREVER);

        k_msgq_peek(&kalmanXQueue, &xPos);
        k_msgq_peek(&kalmanYQueue, &yPos);
        k_msgq_peek(&rssiXQueue, &rssiXPos);
        k_msgq_peek(&rssiYQueue, &rssiYPos);
        printk("%f\r\n", xPos);
        printk("%f\r\n", yPos);
        printk("%f\r\n", rssiXPos);
        printk("%f\r\n", rssiYPos);

		uint8_t xPixelsKalman = 33 + (uint8_t)((xPos / 3.f) * 64);
        uint8_t yPixelsKalman = (uint8_t)((yPos / 4.f) * 64);
       
        uint8_t xPixelsRssi = 33 + (uint8_t)((rssiXPos / 3.f) * 64);
        uint8_t yPixelsRssi = (uint8_t)((rssiYPos / 4.f) * 64);
        
        lv_obj_set_pos(circle, xPixelsKalman - 6, yPixelsKalman - 3);
        
        lv_obj_set_pos(rssiCircle, xPixelsRssi - 6, yPixelsRssi - 6); 
		lv_timer_handler();
        k_msleep(10);
    }
}

K_THREAD_DEFINE(commsId, 1024, comms_thread, NULL, NULL, NULL, 1, 0, 0);
K_THREAD_DEFINE(analysisId, 8192, analysis_thread, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(screenId, 2048, screen_thread, NULL, NULL, NULL, 7, 0, 0);
