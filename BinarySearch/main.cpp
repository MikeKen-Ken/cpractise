#include <chrono>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

int getIndexByBinarySearch(int target, std::vector<int> numbers)
{
    float low = 0;
    float high = numbers.size() - 1;
    float mid = std::ceil((high - low) / 2);
    float searchSize = high - low;

    while (searchSize > 1)
    {
        searchSize = high - low;
        if (numbers[mid] == target)
        {
            return mid;
        }
        else if (numbers[mid] > target)
        {
            if (numbers[low] == target)
            {
                return low;
            }
            high = mid;
            mid = std::ceil((mid - low) / 2);
        }
        else
        {
            if (numbers[high] == target)
            {
                return high;
            }
            low = mid;
            mid = low + std::ceil((high - low) / 2);
        }
    }
    return -1;
}

int main()
{
    // std::cout << "Please enter alist of 20 integer numbers: ";
    int num;
    std::vector<int> numbers = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    // while (std::cin >> num && numbers.size() != 10)
    // {
    //     numbers.push_back(num);
    // }

    // std::sort(numbers.begin(), numbers.end());

    // std::cout << "The sorted list is: ";
    // for (const auto &i : numbers)
    //     std::cout << i << ' ';

    std::cout << "\nPlease enter a integer value in numbers";
    // std::cin.clear();
    // std::cin.ignore(10000, '\n');
    int target;

    std::cin >> target;
    std::cout << target;
    int index = getIndexByBinarySearch(target, numbers);
    std::cout << "\nthe num " << target << " index is " << index;
    return 0;
}