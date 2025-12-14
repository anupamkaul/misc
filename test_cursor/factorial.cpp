#include <iostream>
#include <iomanip>
#include <limits>

// Template function to calculate factorial recursively.
template <typename T>
T factorial(T n) {
    if (n <= 1)
        return 1;
    return n * factorial(n - 1);
}

int main() {
    const int test_val = 10;
    long double result = factorial(static_cast<long double>(test_val));
    std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
    std::cout << "Factorial of " << test_val << " is:\n" << result << std::endl;
    return 0;
}

