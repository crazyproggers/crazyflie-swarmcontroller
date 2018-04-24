#ifndef OBSERVER_H
#define OBSERVER_H

// Google: "observer pattern"

class Observer {
public:
    Observer()                           = default;
    Observer(const Observer&)            = delete;
    Observer& operator=(const Observer&) = delete;

    virtual void update() = 0; // update state of observer
};


class Subject {
    std::vector<std::shared_ptr<Observer>> m_observers;

public:
    Subject()                          = default;
    Subject(const Subject&)            = delete;
    Subject& operator=(const Subject&) = delete;

    // Attach new observer
    void attach(std::shared_ptr<Observer> observer) noexcept {
        m_observers.push_back(observer);
    }

    // Notify all observers that Subject has been changed its state
    void notify() noexcept {
        for (auto observer: m_observers)
            observer->update();
    }
};

#endif // OBSERVER_H