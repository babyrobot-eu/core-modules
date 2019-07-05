# Zeno Interaction - Maven
The interaction with zeno over a wifi using a Java based application needs a JDK 7

## Software needed
* Netbeans/Command line
* [Apache Maven](https://maven.apache.org/) 
* JDK 7 (Mechio is a robot control framework, it uses Maven build tool and Java 7)

## Dependencies
  1. All the necessary built in, dependencies, repositories etc can be added to the pom.xml file. 
  2. Maven repo adds all the necessary dependencies to the local repository (.m2/repository) 
  3. The advantage of using maven is that the runtime and compile time dependencies are added from local repository or from the online repo.

## To create a Maven project
[Maven Project](http://www.vogella.com/tutorials/ApacheMaven/article.html#maven_installation)


## Run the project from command line
Go to the directory which contains the pom.xml file

To install all the necessary files to the local repository 
```shell
mvn install
```
To ensure that the build target is removed before a new build
Run: mvn clean install 

To compile your Java sources run command 
Run: mvn compile

To run the actual task or the program 
Run: mvn exec:java -Dexec.mainClass='zeno_mvn.ZenoInteraction'

