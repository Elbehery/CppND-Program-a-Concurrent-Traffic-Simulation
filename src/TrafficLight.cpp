#include <iostream>
#include <random>
#include <mutex>
#include <future>
#include "TrafficLight.h"

/* Implementation of class "MessageQueue" */

 
template <typename T>
T MessageQueue<T>::receive()
{ 
    std::unique_lock<std::mutex> lck(_mutex);
    _cond.wait(lck,[this](){
        return !_queue.empty();
    });

    T t = std::move(_queue.back());
    _queue.pop_back();
    return t;
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    std::lock_guard<std::mutex> lck(_mutex);
    _queue.push_back(std::move(msg));
    _cond.notify_one();
}

/* Implementation of class "TrafficLight" */

TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::RED;
    _messages = std::make_shared<MessageQueue<TrafficLightPhase>>();
}

void TrafficLight::waitForGreen()
{
    while (true) {
        if(_messages.get()->receive() == TrafficLightPhase::GREEN){
            return;
        }
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    return _currentPhase;
}

void TrafficLight::simulate()
{
    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}


// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(4000, 6000);
    double cycleDuration = dis(gen);

    std::chrono::time_point<std::chrono::system_clock> lastUpdate = std::chrono::system_clock::now();
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();

        if (timeSinceLastUpdate >= cycleDuration){
            
            std::unique_lock<std::mutex> lck(_mtx);
            std::cout << "Traffic light #" << _id << "::cycleThroughPhases: thread id = " << std::this_thread::get_id() << std::endl;
            lck.unlock();
            
            if(_currentPhase == TrafficLightPhase::RED){
                _currentPhase = TrafficLightPhase::GREEN;
                std::cout << "Traffic light is now: GREEN" << std::endl;
            }
            else{
                _currentPhase = TrafficLightPhase::RED;
                std::cout << "Traffic light is now: RED" << std::endl;
            }

            TrafficLightPhase msg = _currentPhase;
            auto has_sent = std::async(std::launch::async,&MessageQueue<TrafficLightPhase>::send, _messages, std::move(msg));
            has_sent.wait();
            cycleDuration = dis(gen);
            lastUpdate = std::chrono::system_clock::now();
        }
    }
}

