#pragma once
#include <string>  // for string class
#include "hopper_mpc/tinyfsm.hpp"

// ----------------------------------------------------------------------------
// Event declarations
//
struct Update : tinyfsm::Event {};

// struct ToCmpr : tinyfsm::Event {};
// struct ToPush : tinyfsm::Event {};
// struct ToRise : tinyfsm::Event {};
// struct ToFall : tinyfsm::Event {};

// ----------------------------------------------------------------------------
// (FSM base class) declaration
//

class StateMachine : public tinyfsm::Fsm<StateMachine> {
  /* NOTE: react(), entry() and exit() functions need to be accessible
   * from tinyfsm::Fsm class. You might as well declare friendship to
   * tinyfsm::Fsm, and make these functions private:
   *
   * friend class Fsm;
   */
 public:
  /* default reaction for unhandled events */
  void react(tinyfsm::Event const&){};

  virtual void react(Update const&);
  // virtual void react(ToCmpr const&);
  // virtual void react(ToPush const&);
  // virtual void react(ToRise const&);
  // virtual void react(ToFall const&);

  virtual void entry(void){}; /* entry actions in some states */
  void exit(void){};          /* no exit actions at all */

 protected:
  static bool s_;
  static bool sh_;
  static double dz_;

 public:
  void ReceiveData(bool s, bool sh, double dz);
};

bool StateMachine::s_ = true;
bool StateMachine::sh_ = false;
double StateMachine::dz_ = 0;

class Cmpr : public StateMachine {
  void entry() override;
  void react(Update const&) override {
    if (dz_ >= 0) {
      transit<Push>();
    };
  }
};

class Push : public StateMachine {
  void entry() override;
  void react(Update const&) override {
    if (s_ == false && sh_ == false) {
      transit<Rise>();
    };
  }
};

class Rise : public StateMachine {
  void entry() override;
  void react(Update const&) override {
    if (dz_ <= 0) {
      transit<Fall>();
    }
  };
  // void react(ToFall const&) override { transit<Fall>(); };
};

class Fall : public StateMachine {
  void entry() override;
  void react(Update const&) override {
    if (sh_ == true) {
      transit<Cmpr>();
    }
  };
};