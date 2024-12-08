函数模板

````cpp
#include "iostream"
template<typename T>

class num{
    private:
    //创建一个 T 类型的 成员变量 value
    //T 在main(){}中会声明
    //如 num<int> intnum(10);
    //就声明了T 为 int;
    T value;
    public:
    c_in (T v) : value(v){
        std::cout<< value << std::endl;
        
    }
    //get_value是可以从模板外调用value值的函数
    get_value(){
        return value;
    }
    
};
````



