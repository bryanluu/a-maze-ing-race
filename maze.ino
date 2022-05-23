#include <map>

namespace Test {
  void sayHello() {
    Serial.println("Hello World!");
  }
}

void sayHello() {
  Serial.println("Hello!");
}

class Dog {
  String name;
  int weight;

public:
  Dog();

  void setName(const String dogsName);
  void setWeight(int dogsWeight);
  void print() const;
};

Dog myDog;

template<class T>
class LinkedList {
  T value;
  LinkedList<T> * next;
  int size;
  
public:
  LinkedList<T>(T item) {
    value = item;
    size = 1;
    next = NULL;
  }

  void insert(T item)
  {
    next = new LinkedList<T>(item);
    size++;
  }
  
  int getSize()
  {
    return size;
  }

  void print()
  {
    LinkedList<T> * curr = this;
    String out;
    for (int i = 0; i < size; i++)
    {
      out += String(curr->value);
      curr = curr->next;
      if (i < size - 1)
      {
        out += " -> ";
      }
    }
    Serial.println(out);
  }

  ~LinkedList<T>()
  {
    delete next;
  }
};

LinkedList<int> list(0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myDog.setName("Barks");
  myDog.setWeight(10);
  list.insert(1);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  myDog.print();
  list.print();
}

Dog::Dog()
{
  
}

void Dog::setName(const String dogsName)
{
  name = dogsName;
}

void Dog::setWeight(int dogsWeight)
{
  weight = dogsWeight;
}

void Dog::print() const
{
  Serial.println("Dog is " + name + " and weighs " + String(weight));
}
