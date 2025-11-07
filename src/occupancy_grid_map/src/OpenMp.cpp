#include<iostream>
#include<chrono>
#include"omp.h"
 
using namespace std;
 
void test()
{
	for (int i = 0; i < 80000; i++)
	{
	}
}
 
int main(int argc, char **argv)
{
	auto start_time = std::chrono::high_resolution_clock::now();

	//指定2个线程
#pragma omp parallel for num_threads(2)
	for (int i = 0; i < 80000; i++)
	{
		test();
	}
	auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
	std::cout << "指定 2 个线程，执行时间: " << duration_ns / 1000000.0 << "ms\n";

 
	//指定4个线程
	start_time = std::chrono::high_resolution_clock::now();
#pragma omp parallel for num_threads(4)
	for (int i = 0; i < 80000; i++)
	{
		test();
	}
	end_time = std::chrono::high_resolution_clock::now();
	duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
	std::cout << "指定 4 个线程，执行时间: " << duration_ns / 1000000.0 << "ms\n";

 
	//指定8个线程
	start_time = std::chrono::high_resolution_clock::now();
#pragma omp parallel for num_threads(8)
	for (int i = 0; i < 80000; i++)
	{
		test();
	}
	end_time = std::chrono::high_resolution_clock::now();
	duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
	std::cout << "指定 8 个线程，执行时间: " << duration_ns / 1000000.0 << "ms\n";
 
	//指定12个线程
	start_time = std::chrono::high_resolution_clock::now();
#pragma omp parallel for num_threads(12)
	for (int i = 0; i < 80000; i++)
	{
		test();
	}
	end_time = std::chrono::high_resolution_clock::now();
	std::cout << "指定 12 个线程，执行时间: " << duration_ns / 1000000.0 << "ms\n";

	//不使用OpenMP
	start_time = std::chrono::high_resolution_clock::now();
	for (int i = 0; i < 80000; i++)
	{
		test();
	}
	end_time = std::chrono::high_resolution_clock::now();
	std::cout << "不使用OpenMP多线程，执行时间: " << duration_ns / 1000000.0 << "ms\n";
}