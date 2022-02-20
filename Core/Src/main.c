/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

// Uh so the current plan is that the sensor values are read with an interrupt
// But then we also implement blocking functions for moving in a direction, or
// checking if there's a wall in a given direction. That can do what they want

// TODO: Function to move 1 square in any direction
// TODO: Function to check if there's a wall in a given direction
// TODO: Check DIP switches for maze algo
// TODO: Once we've solved the maze, go back to the beginning, and run it
// TODO: Any number of bugs arising from the fact that I don't know C

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAZE_SIZE 10
// 0 - Basic A*; 1 - A*, but tries to go center; 2 - A²*, 3 - Augment A* and extra huristic
#define MAPPING_MODE 3

#define END_X 5
#define END_Y 5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// A maze
// Uh
// maze[Y][X][1: vertical; 0: horizontal]
// And the number represents the confidence
// So 0 is we have no idea, and positive is wall, and negative is no wall
int maze[11][11][2];

// Aahhhah
typedef struct Node Node;

struct Node {
	int x;
	int y;
	int closed; //Boolean but this is C :(
	int distance; // The length of the path up to this point, this.last.distance + 1
	int guess; // The heuristic, usually Manhattan to goal with no walls
	int score; //The guess + distance (depending on algo) (lower is better)
	Node* last; //The parent node, the node from which we discovered this node
};

// The list of nodes
// Sorted such that closed nodes are 0..closedNodes-1
// And nodes after that increase in score
// This is the canonical list of nodes, order doesn't matter
// All other nodes are pointers into nodesStatic
Node nodesStatic[MAZE_SIZE*MAZE_SIZE];
// A*'s working list of nodes. 0..closedNodes-1 are closed, and closedNodes..numNodes-1 are open
Node *nodes[MAZE_SIZE*MAZE_SIZE];
int closedNodes = 0;
int numNodes = 0;

//let robot (Needs to keep track of inter-tile location, e.g. encoder counts)
Node *current; //The node the robot is currently at (or that A* thinks we're at)
Node *goal; //The current goal node
//let state (Might be needed to track what we're doing. Might be fine a while true loop and blocking functions)
Node *backPath[MAZE_SIZE*MAZE_SIZE]; // Used to store the path along which we backtrack to get to goal
int backPathLength = 0;

// The main path, once we've solved the maze, the route we use
Node *mainPath[MAZE_SIZE*MAZE_SIZE];
int pathLength = 0;

//Heuristic function
int h (int x, int y) {
	return abs(END_X - x) + abs(END_Y - y);
}

void insertAt(Node **arr, int len, int i, Node* n) {
	for (int j = len; j > i; j --) {
		arr[j] = arr[j - 1];
	}
	arr[i] = n;
}

void removeAt(Node **arr, int len, int i) {
	for (int j = i; j < len - 1; j ++) {
		arr[j] = arr[j + 1];
	}
}

void addNodeIfNotExists(int x, int y) {
	//Check if we've seen the node before
	for (int i = 0; i < numNodes; i++) {
		if (nodes[i]->x == x && nodes[i]->y == y) {
			//We don't have to ever update the score on a node
			//Because our heuristic is nice (just distance)
			return;
		}
	}

	Node *n = &nodesStatic[numNodes];
	n->x = x;
	n->y = y;
	n->last = current;
	n->guess = h(x, y);
	n->distance = current->distance + 1;
	if (MAPPING_MODE == 0) {
		n->score = n->distance + n->guess;
	}else if (MAPPING_MODE == 1 || MAPPING_MODE == 3) {
		n->score = n->distance + n->guess*10;
	}

	/*//Just assertions
	assertEq(nodes[closedNodes].closed, false, "AHHHH");
	if (closedNodes < nodes.length - 1) {
		assertEq(nodes[closedNodes + 1].closed, false, "B");
	}else {
		console.assert(false, "testing");
	}
	if (closedNodes >= 1) {
		assertEq(nodes[closedNodes - 1].closed, true, "B");
	}*/

	//Sort the new node in
	for (int i = closedNodes; i < numNodes; i++) {
		if (nodes[i]->score > n->score) {
			//nodes.splice(i, 0, n);
			//arr, length, index to insert, item
			insertAt(nodes, numNodes, i, n);
			numNodes++;
			return;
		}
	}
	//Otherwise, add the node to the end
    //	nodes.splice(nodes.length, 0, n);
    //	insertAt(nodes, numNodes, numNodes, n);
	nodes[numNodes] = n;
	numNodes++;
}

int checkDone() {
	return current->x == END_X && current->y == END_Y;
}

// Once we've solved the maze, converts node tree into `mainPath` (a list)
void createPath() {
	mainPath[0] = current;
	Node *previous = current;
	pathLength = 1;
	while (previous->last->distance >= 0) {
		// path.unshift(backtrack.last);
		insertAt(mainPath, pathLength, 0, previous->last);
		pathLength++;
		previous = previous->last;
	}
}



//Update the known information about the maze with what we know
void updateMaze() {
	//TODO: Scan the maze

	//If there's not a wall to each of our sides, add a new node there
	if (!maze[current->y][current->x][0]) {
		addNodeIfNotExists(current->x, current->y - 1);
	}
	//Left
	if (!maze[current->y][current->x][1]) {
		addNodeIfNotExists(current->x - 1, current->y);
	}
	//Down
	if (!maze[current->y + 1][current->x][0]) {
		addNodeIfNotExists(current->x, current->y + 1);
	}
	//Right
	if (!maze[current->y][current->x + 1][1]) {
		addNodeIfNotExists(current->x + 1, current->y);
	}
}

void updateGoal () {
	if (closedNodes >= numNodes) {
		return;
	}

	if (MAPPING_MODE == 1 || MAPPING_MODE == 2) {
		goal = nodes[closedNodes];
	}else {
		//If there's one adjacent node, go there, otherwise go to A*
		Node *adjacentNode = NULL;

		for (int i = 0; i < numNodes; i++) {
			//If it's open and adjacent
			if (nodes[i]->last == current && !nodes[i]->closed) {
				// If we already have one
				if (adjacentNode != NULL) {
					//Then we want to ignore it
					adjacentNode = NULL;
				}else {
					adjacentNode = nodes[i];
				}
			}
		}

		if (adjacentNode == NULL) {
			goal = nodes[closedNodes];
		}else {
			goal = adjacentNode;
		}
	}
}

void createBackPath() {
	//finding backPath only needs to run once
	// Create a path.
	//      Find a common parent of current and goal
	//	backPath[0] = current;
	//	backPath[1] = goal;
	memcpy(&backPath[0], current, sizeof(Node));
	memcpy(&backPath[1], goal, sizeof(Node));
	int numMid = 1; //The number of nodes that we go up before going back down.
	backPathLength = 2;

	const int genDiff = abs(current->distance - goal->distance);

	for (int i = 0; i < genDiff; i++) {
		if (current->distance > goal->distance) {
			// parentCur.push(parentCur[parentCur.length-1].last);
			// backPath.splice(numMid, 0, backPath[numMid - 1].last);
			insertAt(backPath, backPathLength, numMid, backPath[numMid - 1]->last);
			numMid++;
			backPathLength++;
		}else {
			// parentGoal.unshift(parentGoal[0].last);
			// backPath.splice(numMid, 0, backPath[numMid].last);
			insertAt(backPath, backPathLength, numMid, backPath[numMid]->last);
			backPathLength++;
		}
	}
	//Walk back up the tree until they're the same
	while (backPath[numMid-1]->last != backPath[numMid]->last) {
		// parentCur.push(parentCur[parentCur.length-1].last);
		// backPath.splice(numMid, 0, backPath[numMid-1].last);
		insertAt(backPath, backPathLength, numMid, backPath[numMid-1]->last);
		numMid++;
		backPathLength++;

		// parentGoal.unshift(parentGoal[0].last);
		// backPath.splice(numMid, 0, backPath[numMid].last);
		insertAt(backPath, backPathLength, numMid, backPath[numMid]->last);
		backPathLength++;
	}
	//Remove the duplicated shared parent, that they both pushed
	// backPath.slice(numMid, 0, backPath[numMid].last);
	removeAt(backPath, backPathLength, numMid);
	backPathLength--;
}

void moveToGoal() {
    // Move back along path
    //      To move one node, move ? per tile
    // backPath is a list of contiguous nodes. First is current, last is goal

	//TODO: Uh spin some motors or something
	// Probably refactor this to call a blocking move function,
	// And continue once we're there, instead of checking if we're there

//    if (backPath[1].x == (robot.x - SQUARE_SIZE/2)/SQUARE_SIZE &&
//        backPath[1].y == (robot.y - SQUARE_SIZE/2)/SQUARE_SIZE) {
//        //Then we've made it
//        backPath.shift(); //Remove the first
//    }
	if (1==1 /*We're there, by some metric*/) {
		//Then we've made it
		removeAt(backPath, backPathLength, 0);
		backPathLength--;
	}

    if (backPathLength == 1) {
        //Then we're actually done,

        current = backPath[0];
        backPathLength = 0;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  // TODO: Check DIP here I think

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
		while (1 == 1) {
			//Then we've solved the maze
			if (pathLength > 0) {
				osDelay(10);
				//TODO: Run the maze in reverse, then forwards
			}else {
				//Go to goal target, if we get there, update
				//TODO: Check if we've made it to the goal
				// if (goal.x == (robot.x - SQUARE_SIZE/2)/SQUARE_SIZE &&
				// goal.y == (robot.y - SQUARE_SIZE/2)/SQUARE_SIZE) {
				// Can probably check if backPathLength == 0 or something
				if (1 == 1 /*TODO*/) {
					//Update where we are
					current = goal;

					//Check if we're done
					if (checkDone()) {
						createPath();
					}else {
						//Update our knowledge of the maze, from the current tile
						// i.e. scan around us
						updateMaze();

						//Close the current node
						current->closed = 1;
						//If this wasn't the next node scheduled to be closed, we need to move it to the closed section of the list
						if (nodes[closedNodes] != current) {
							Node *lastValue = nodes[closedNodes];
							for (int i = closedNodes + 1; i < numNodes; i++) {
								Node *tmp = nodes[i];
								nodes[i] = lastValue;
								lastValue = tmp;
								if (tmp == current) {
									//Then we're done
									break;
								}
							}
							nodes[closedNodes] = current;
						}
						closedNodes++;

						// Update goal
						//  get a new goal
						updateGoal();
					}
				}else {
					//Move to previously discovered `goal` tile
					if (backPathLength > 0) {
					  createBackPath();
					}
					moveToGoal();
				}
			}
		}

	    osDelay(1);
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
