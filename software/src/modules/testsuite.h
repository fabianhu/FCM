/*
 * testsuite.h
 *
 * Created: 25.04.2014 16:55:59
 *
 * (c) 2014-2015 by Fabian Huslik
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */  


#ifndef TESTSUITE_H_
#define TESTSUITE_H_

#if TEST_RUN == 1

#define MAXTESTCASES 20 // one more than the max "case" ID.

extern uint8_t testcase; // global test case number
extern uint8_t TestResults[MAXTESTCASES]; // test result array (0= OK)
extern uint8_t TestProcessed[MAXTESTCASES]; // test passed array (number of processed assertions


#define assert(X) do{			\
	OS_DISABLEALLINTERRUPTS;	\
	if(!(X))					\
	{							\
		TestResults[testcase]++;\
		asm("breakpoint");		\
	}							\
	TestProcessed[testcase]++;	\
	OS_ENABLEALLINTERRUPTS;		\
}while(0)

#define epsilon 10.0E-3


#define fequal(x,y) (fabs((x) - (y)) < epsilon)
#define assertfequal(x,y) assert(fequal(x,y))
#define radgra 57.295779513f

void test_run(void);

#endif

#endif /* TESTSUITE_H_ */