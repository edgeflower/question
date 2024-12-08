# C++

## 3 函数提高

### 3.1 函数默认参数

语法： `返回值类型   函数名 （参数=默认值）{}`

__若某位置有了默认参数，那么从这个位置往后，从左到右都必有默认值__

``` c++
int func(int a, int b=10 , int c=20){return a+b+c;}
//若某位置有了默认参数，那么从这个位置往后，从左到右都必有默认值
int main(){
cout << func(10)<<endl;
return 0;
}
```

- __如果函数声明有默认参数，函数实现不能有默认参数__

### 3.2 函数占位参数

语法`返回值类型 函数名 （数据类型）{}`

__示例__:

````C++
//函数占位参数，占位参数也可以有默认参数
void func(int a,int){
    cout << "this is func" << endl;
}
int main (){
    func(10,10);//占位参数必须补
    system(“pause”);
}
````

### 3.3 函数重载

#### 3.3.1 函数重载概述



````c++
//函数重载需要函数都在同一个作用域下
//函数名相同
//函数参数可以是个数不同也可以是类型不同
int func()
{}

````

#### 3.3.2 函数重载的注意事项

- 引用作为重载的条件

```c++
void func(int %a)
{
    
}
void func (const int &a)
{
    
}
//2.函数重载碰到函数默认参数
void func2(int a,int = 10)
{
    
}
void func2(int a)
{}
int main(){
    int a = 10;
    func(a);//调用无const
    func(10);//调用有const
   
    //func2(10)://碰到默认参数产生歧义，需要避免
    return ;
}
```

## 4  类和对象

c++面向对象的三大特性：封装、继承、堕胎

### 4.1 封装

#### 4.1.1封装的意义

```c++
//设计一个圆类，求圆的周长
//圆求周长的公式：2 * PI*半径

//class 代表设计一个类，类后面紧跟着的是类名称
class Circle
{
 public:
    //访问权限
    //公共权限
	int m_r;
    //行为
    //获取圆的周长
    double calculateZC(){
        return 2*PI*m_r;
    }
    int main(){
        //通过圆类创建具体的圆（对象）
        //实例化	(通过一个类创建一个对象的过程)
        Circle c1;
        //给圆对象的属性进行赋值
        c1.m_r = 10;
        //2*PI*10=62.8
        cout << "圆的周长为："<<c1.calculateZC（）<<endl;
    return 0;
    
}
```



#### 4.1.2 封装的权限控制

``` c++
#include<iostream>
using namespace std;

//访问权限
//三种
//公共权限 public	成员 类内可以访问 类外不可以访问 
//保护权限 protected	成员 类内可以访问 类外不可以访问 儿子可以访问父亲中的保护内容
//私有权限	类外不可以访问 儿子不可以访问父亲的私有内容
class Person
    // 公共权限
    string m_Name;	// 姓名
protected: 
	//私有权限
	int m_Password;//银行卡密码

public:
void func()
{
    m_Name = “张三”;
    m_Car = "拖拉机";
    m_Password = 123456;
    
};
int main(){
    //实例化具体对象
    Person p1;
    p1.m_Name = "李四";
    //p1.m_Car = "奔驰";//保护私有内容在类外访问不到
    //p1.m_Password = 123;//私有内容，在类外访问不到
    return 0 ；
        
}

```

#### 4.1.3struct和class的区别

__struct__默认权限是公共

__class__默认是私有

#### 4.1.4 成员设置为私有

```c++
#define _CRT_SECURE_NO_WARNINGS
#include<iostream>
using namespace std;
#inlude <string>
//成员属性设置私有
//1、可以自己控制读写权限
//2、对于写可以检测数据有效性

//人类
class Person
{
    public:
    //设置姓名
    void setName(string name){
        m_Name = name;
    }
    //获取姓名
    string getName()
    {
        return m_Name;
    }
    //获取年龄
    int getAge()
    {
        return m_Age
    }
    //设置年龄（0~150）
    void setAge(int age){
        if(age< 0||age> 150){
            cout << "年龄"<<age << "输入有误，赋值失败"<<endl;
        }
    }
    //设置偶像
    void setIdol(string idol){
        m_Idol=idol
    }
    private:
    string m_Name;//姓名	可读可写
    int m_Age = 18;//年龄	只读	可读可写
    string m_Idol;//偶像	只写
};
int main(){
    Person p;
    //姓名设置
    p.setName(张三)；
    //获取姓名
        cout << "姓名："<<p.getName()<< endl;
    //获取年龄
    cout << "年龄"<<p.getAge()<<endl;
     
    //偶像设置
    p.setIdol("小明");//只写
    //cout<<"偶像"<<p.getIdol()<<endl;//只写状态	外界访问不到
    
}
```

- 案例

  - __设计立方体__

    ```c++
    class
    ```

  - **点和圆关系案例**

````
````

### 4.2对象的初始化和清理

#### 4.2.1 构造函数和析构函数

- 构造函数`类名(){}`
  1. 不用加返回值，也不用写void
  2. 函数名与类名相同
  3. 构造函数可有参数，因此可以重载
  4. 程序在调用对象时会自动调用构造，无需手动调用，且只调用一次

- 析构函数`~类名(){}`
  1. 不用加返回值，也不用写void
  2. 函数名与类名相同，在名称前面加~
  3. 构造函数不可以有参数，因此不可以重载
  4. 程序在调用对象时会自动调用析构，无需手动调用，且只调用一次

#### 4.2.2 构造函数的分类与调用

````c++
#include <iostream>                                                                            
using namespace std;                                                                           
class Person {                                                                                 
public:                                                                                        
    Person()                                                                                   
    {                                                                                          
        cout << "默认构造函数"  << endl;                                                             
    }                                                                                          
    Person (int a ,int Hight)                                                                  
    {                                                                                          
        cout << "含参构造函数" << endl;                                                              
        age = a;                                                                               
        //将int mian 函数中传入的Hight值赋给m_Hight                                                      
        m_Hight =  new int (Hight);                                                            
    }                                                                                          
    Person( const Person &p) {                                                                 
        age = p.age ;                                                                          
        //将m_Hight 的值储存在新开辟的储存空间m_Hight中                                                       
        //防止 new int (m_Hight)被析构函数（delete）清除空间后被二次清除空间造成错误                                    
        m_Hight = new int( *p.m_Hight);                                                        
        cout << age  <<"+" <<*m_Hight  <<endl;                                                 
    }                                                                                          
    ~Person()                                                                                  
    {    //析构函数（delete）清除空间 m_Hight                                                            
        //main函数出现几次Person就会被调用                                                                
        delete m_Hight;                                                                        
        cout <<"调用析构函数" << endl;                                                               
    }                                                                                          
private:                                                                                       
    //要在private 或新的public   下写名public函数中声明的值                                                   
    //如 age m_Hight                                                                            
    int age{} ;                                                                                
    int *m_Hight{};                                                                            
};                                                                                             int main() {
    Person p2 (10 , 160);
    Person p1(p2);
    return 0 ;
}
````

#### 4.2.3 初始化列表

````C++
#pragma once//只执行一次头文件
#include<iostream>
using namespace std;
class Person {
public://要写public,类默认不能访问内部成员m_A m_B m_C
    //初始化列表
    Person(int a ,int b ,int c): m_A(a) , m_B(b) , m_C(c)
    {

    }

    int m_A;
    int m_B;
    int m_C;
};
void test() {
    Person p(10,20,30);
    cout << "m_A = " << p .m_A << endl;
    cout << "m_A = " << p.m_B << endl;
    cout << "m_A = " << p.m_C<< endl;
}
//上面是头文件T.h
//上面是头文件T.h
//上面是头文件T.h

#include <iostream>
#include"T.h"
int main()
{test();
    std::cout << "Hello, World!" << std::endl;
    return 0;
}


````

#### 4.2.4 类对象作为类成员

````c++
#pragma once
#include <iostream>
#include <string>
#include <utility>
using namespace std;

//手机类

class Phone
{
public:
    //传入PName,并储存到 m_PName
    explicit Phone(string PName )
    {
        m_PName = std::move(PName);
        cout << "调用Phone" << endl;
    }

        string m_PName;
};

//人类

class Person
{
public:
    Person(string Pname , string Mname) :m_phone(std::move(Pname)) , m_Name(std::move(Mname))
    {
        cout << "调用Person" << endl;
    }
        Phone m_phone;  //连接Person类与Phone类的桥梁
        string m_Name;
};

//测试函数



void test()
{
        Person p ("安兴","lulu");
        cout << p.m_Name << "拿着" << p.m_phone.m_PName<< "牌手机。" << endl;
        //p.m_phone.m_PName 调用Person中 Phone类成员的存储在 string类型的m_phone的值
}
   
//上面是头文件t.h
//上面是头文件t.h
//上面是头文件t.h

#include"t.h"
int main()
{
 test();
    return 0;
}

````

#### 4.1.5 静态成员变量



````c++
#pragma once
#include <iostream>
#include <string>
using namespace std;
class Person {
public:
    static  int m_A;
    static  int m_B;

    //静态成员也有权限限制    ，去除private的注释符即可看到m_C不可访问。
//private:
    static  double m_C;



};
 int Person:: m_A = 100;
 int Person:: m_B = 1020;
 double Person :: m_C = 120;

void test() {
    Person p;
    // 静态成员函数中每个成员指向一个值，其中一个成员改变，其他所有成员都改变。
    p.m_A = 2;
    cout << p.m_A << "+" << p.m_B <<endl;
    cout << p.m_C << endl;;
    //静态成员有两种访问方式
    // p.m_C        （通过p.访问）
    //Person::m_C   (直接访问)
cout  << Person ::m_A << endl;
}
// 上面是头文件j.h
// 上面是头文件j.h
// 上面是头文件j.h

#include "j.h"

int main() {
   test();

    return 0;
}
````



### 4.3 C++对象模型和this指针

#### 4.3.1 成员变量和成员函数分开存储

````c++
#pragma once
#include <string>
#include <iostream>
using namespace std;

class Person {

};
void  test () {

    Person p;
    cout << "输出空类的储存空间大小" << sizeof(p) << endl;
    //空对象的类占用一个字节
    //为了可以区分空对象占用内存的位置
}
class Phone {

    int a;

};
void test01() {

    Phone p;
    cout << "输出含有int函数的储存空间大小" << sizeof(p) << endl;
    //非静态成员变量储存空间在类上

}
class PUPU {

    int static a;

};


void test02() {

    PUPU p;
    cout << "输出含有static函数的储存空间大小" << sizeof(p) << endl;
    //静态成员储存空间在类外
}

//上面是头文件m.h
//上面是头文件m.h
//上面是头文件m.h


#include "m.h"

int main()
{
   test();
    test01();
    return 0;
}

````



#### 4.3.2 this指针概念

**相当重要**

**this 指针指向被调用的成员函数所属的对象**

````c++
#pragma once
#include <iostream>
using namespace std;
//this  指针的调用
class Person1
{
public:
   explicit Person1(int age)
   {
      //命名不规范，编译器让你好看；
       age =age;
   }
       int  age;
   //这样Person中传入的age数据不会传到 int类型的age 中。
};
//解决方法一   命名规范

class Person2
{
public:
   explicit Person2(int age)
   {
       m_Age = age ;

   }
       int m_Age;
};

//方法二    this指针

class Person3
{
public:
    explicit Person3(int age)
   {
       this->age = age ;

   }
       int age;
    Person3&  PersonAddAge( Person3 &p )
    {
       this->age += p.age;
        //返回对象本身。
       return *this;
   }
};
 void test ()
{
    Person3 p(18);
    Person3 p1(p);
    //p1.PersonAddAge(p).PersonAddAge(p);是一个函数单独一行 行使。
    //调用了 p1.PersonAddAge(p).PersonAddAge(p);。
    //这里，PersonAddAge 函数被连续调用了两次，每次都传入了对象 p。
    //第一次调用后，p1 的年龄将增加 p 的年龄（18岁），然后返回 p1 的引用。
    //第二次调用再次将 p 的年龄加到 p1 上。
    // 因此，执行这两次调用后，p1 的年龄将是
    //  18 + 18 + 18 = 54 岁。
    p1.PersonAddAge(p).PersonAddAge(p);
    //套娃
    cout << "年龄" << p1.age;

 }

//上面是一个头文件k.h
//上面是一个头文件k.h
//上面是一个头文件k.h

#include"k.h"
int main()
{
    test();
    return 0;
}

````





#### 4.3.3 空指针访问成员函数

````c++
#pragma once
#include<iostream>
using namespace std;

//使用空指针访问成员函数
//不可以访问有空指针的成员函数！！！！！
class Person
{

    public:

    void ShowName()
    {

        cout << "这是一个class 类" << endl;
    }
    void ShowAge()
    {

        //加入下面三行代码会避免出现信号: SIGSEGV (Segmentation fault)的问题。

        if(this ==NULL)
            {
                return;
            }
            //信号: SIGSEGV (Segmentation fault)的原因是传入的指针为空；

            cout << m_Age;

    }
            int m_Age = NULL;




};
void test() {

Person *p = nullptr;

    p->ShowAge();

    p->ShowName();
    
//上面是头文件f.h
//上面是头文件f.h
//上面是头文件f.h

    #include "f.h"
int main()
{

test();
    return 0;
}
````

#### 4.3.4 成员函数作友元

~~~~c++
#pragma once
#include<iostream>
#include <string>
using namespace std;
//在类内声明
//建筑物类
class Building
{friend void visit();
    public:
        Building();
        string m_SittingRoom;

        //building 指针指向Building 类
        //维护Building 的指针
    ~Building() = default;

    private:

        string m_BedRoom;



};
//好朋友类
class GoodGay {
    public:
        GoodGay();
        //添加析构函数避免内存泄漏
        ~GoodGay(){
            delete building;
        }

        void visit () const;
        void visit1 ();

    Building *building;



};

//在类外赋值。
//实现类外成员函数。
inline Building::Building() {
    m_SittingRoom = "客厅" ;
    m_BedRoom = "卧室" ;
}
 inline  GoodGay::GoodGay() {

    building = new Building;
}

//1  在这个函数中，const关键字表示这个函数不会修改任何成员变量，
//这使得函数可以被const对象调用，并且可以被const成员函数调用。
//
//2  如果你想要确保visit()函数被内联，你可以使用inline关键字
void inline  GoodGay:: visit() const {

    cout << building->m_SittingRoom ;


}

void   inline  GoodGay:: visit1()  {

}
void inline  test() {

    GoodGay p;
    p.visit();

}

//上面是头文件g.h
//上面是头文件g.h
//上面是头文件g.h

#include "g.h"
int main()
{
test ();
    return 0;
}

~~~~

### 4.4 运算符重载

#### 4.4.1加号运算符重载

~~~~c++
#pragma once
#include <string>
class Person {
public:
    int m_A=10;
    int m_B=20;
    //operator+  函数要在类内使用
    Person operator+ (Person &p) const {
        Person temp;
        //this指针默认使用一次，即 在this->m_A  + m_A中this只会作用于m_A;
        temp.m_A = this->m_A +p.m_A;
        temp.m_B = this->m_B +p.m_B;
        return temp ;
    }
};
void test() {
    Person p1;
    Person p2;
    //使用operator+ 函数后可以直接p3 = p1 + p2;
    //而未使用不可以这用做；
    Person p3 = p1 + p2;
    //返回temp.m_A的值
    std::cout << p3.m_A << std::endl;
    //返回temp.m_B的值
    std::cout << p3.m_B << std::endl;
    //返回m_A的值
    std::cout << p2.m_A << std::endl;
}

//上面是头文件d.h

#include "d.h"
int main()
{
   test();
    return 0;
}



~~~~

#### 4.4.2 左移运算符

````c++
#pragma once
#include <iostream>
using namespace std;
//左移运算符重载

class Person {
public:
    int m_A;
    int m_B;


    };
//ostream & operator返回输出流
//这行代码意思是 cout << p
//输出的是
ostream & operator << (ostream & cout ,Person &p) {
 cout << p.m_A << p.m_B;
    return cout;
     }

void test() {

Person p;
    p.m_A = 20;
    p.m_B = 50;
cout << p ;

}

//头文件s.h
//头文件s.h
//头文件s.h

#include"s.h"
int main()
{
    test();
    return 0;
}
````

### 4.5 继承

#### 4.5.1继承

语法`class 子类 : 继承方式 父类` 

~~~~c++
#pragma once
#include <iostream>
#include <string>
//继承
class BasePage {
public:
void header() {

    std::cout << "首页 、公开课" << std::endl;
}
void footer() {

    std::cout << "帮助中心" << std::endl;
}
void heade() {

    std::cout << "首页 、公开课" << std::endl;
}
void head() {

    std::cout << "首页 、公开课" << std::endl;
}

};

//java 类继承 BasePage 类的public中包含的成员
class java : public BasePage {
    void head() {
        std::cout << "首页 " << std::endl;
    }
};
void test() {


    java ja;
    ja.footer();
    ja.heade();
}
~~~~



