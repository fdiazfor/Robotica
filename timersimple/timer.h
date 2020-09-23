#ifndef TIMER_H
#define TIMER_H

#include <thread>
#include <chrono>
#include <functional>
#include <future>
#include <cstdio>
#include <ratio>
#include <iostream>

class Timer
{
    public:

        Timer(){t1 = std::chrono::high_resolution_clock::now();};

        template <class callable>        
        void connect(callable&& f)
        {
			std::thread([=]()
            {
                while(true)
                {
					if(go.load())
						std::invoke(f);
                    std::this_thread::sleep_for(std::chrono::milliseconds(period.load()));
                }
            }).detach();
        };
        
        void start(int p)
        {
			period.store(p);
			go.store(true);
        };
        
        void stop() { go.store(!go); };
		void setPeriod(int p) { period.store(p); };
		int getPeriod() { return period.load(); };
		auto getTimePassed()
        {
            auto t2 = std::chrono::high_resolution_clock::now();
            auto execTime = (t2 - t1).count();
            return execTime;
		};
        
    private:
        std::atomic_bool go = false;
		std::atomic_int period = 0;
        std::chrono::high_resolution_clock::time_point t1;
    
};

#endif // TIMER_H
