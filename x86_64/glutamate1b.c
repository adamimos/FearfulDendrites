/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__glutamate1b
#define _nrn_initial _nrn_initial__glutamate1b
#define nrn_cur _nrn_cur__glutamate1b
#define _nrn_current _nrn_current__glutamate1b
#define nrn_jacob _nrn_jacob__glutamate1b
#define nrn_state _nrn_state__glutamate1b
#define _net_receive _net_receive__glutamate1b 
#define state state__glutamate1b 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define gnmdamax _p[0]
#define gampamax _p[1]
#define e _p[2]
#define Voff _p[3]
#define Vset _p[4]
#define decayampa _p[5]
#define decaynmda _p[6]
#define xloc _p[7]
#define yloc _p[8]
#define tag1 _p[9]
#define tag2 _p[10]
#define inmda _p[11]
#define iampa _p[12]
#define gnmda _p[13]
#define local_v _p[14]
#define local_ca _p[15]
#define A _p[16]
#define B _p[17]
#define gampa _p[18]
#define dampa _p[19]
#define dnmda _p[20]
#define ica _p[21]
#define cai _p[22]
#define DA _p[23]
#define DB _p[24]
#define Dgampa _p[25]
#define Ddampa _p[26]
#define Ddnmda _p[27]
#define v _p[28]
#define _g _p[29]
#define _tsav _p[30]
#define _nd_area  *_ppvar[0]._pval
#define _ion_cai	*_ppvar[2]._pval
#define _ion_ica	*_ppvar[3]._pval
#define _ion_dicadv	*_ppvar[4]._pval
#define diam	*_ppvar[5]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(_ho) Object* _ho; { void* create_point_process();
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt();
 static double _hoc_loc_pnt(_vptr) void* _vptr; {double loc_point_process();
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(_vptr) void* _vptr; {double has_loc_point();
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(_vptr)void* _vptr; {
 double get_loc_point_process(); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 0, 0
};
 /* declare global and static user variables */
#define gama gama_glutamate1b
 double gama = 0.08;
#define icaconst icaconst_glutamate1b
 double icaconst = 0.1;
#define n n_glutamate1b
 double n = 0.25;
#define tau2 tau2_glutamate1b
 double tau2 = 2;
#define tau1 tau1_glutamate1b
 double tau1 = 50;
#define taudnmda taudnmda_glutamate1b
 double taudnmda = 200;
#define taudampa taudampa_glutamate1b
 double taudampa = 200;
#define tau_ampa tau_ampa_glutamate1b
 double tau_ampa = 2;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "tau1_glutamate1b", "ms",
 "tau2_glutamate1b", "ms",
 "tau_ampa_glutamate1b", "ms",
 "n_glutamate1b", "/mM",
 "gama_glutamate1b", "/mV",
 "taudampa_glutamate1b", "ms",
 "taudnmda_glutamate1b", "ms",
 "gnmdamax", "nS",
 "gampamax", "nS",
 "e", "mV",
 "A", "nS",
 "B", "nS",
 "gampa", "nS",
 "inmda", "nA",
 "iampa", "nA",
 "gnmda", "nS",
 "local_v", "mV",
 0,0
};
 static double A0 = 0;
 static double B0 = 0;
 static double delta_t = 0.01;
 static double dnmda0 = 0;
 static double dampa0 = 0;
 static double gampa0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "icaconst_glutamate1b", &icaconst_glutamate1b,
 "tau1_glutamate1b", &tau1_glutamate1b,
 "tau2_glutamate1b", &tau2_glutamate1b,
 "tau_ampa_glutamate1b", &tau_ampa_glutamate1b,
 "n_glutamate1b", &n_glutamate1b,
 "gama_glutamate1b", &gama_glutamate1b,
 "taudampa_glutamate1b", &taudampa_glutamate1b,
 "taudnmda_glutamate1b", &taudnmda_glutamate1b,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 static void _hoc_destroy_pnt(_vptr) void* _vptr; {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[6]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"glutamate1b",
 "gnmdamax",
 "gampamax",
 "e",
 "Voff",
 "Vset",
 "decayampa",
 "decaynmda",
 "xloc",
 "yloc",
 "tag1",
 "tag2",
 0,
 "inmda",
 "iampa",
 "gnmda",
 "local_v",
 "local_ca",
 0,
 "A",
 "B",
 "gampa",
 "dampa",
 "dnmda",
 0,
 0};
 static Symbol* _morphology_sym;
 static Symbol* _ca_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 31, _prop);
 	/*initialize range parameters*/
 	gnmdamax = 1;
 	gampamax = 1;
 	e = 0;
 	Voff = 0;
 	Vset = -60;
 	decayampa = 0.5;
 	decaynmda = 0.5;
 	xloc = 0;
 	yloc = 0;
 	tag1 = 0;
 	tag2 = 0;
  }
 	_prop->param = _p;
 	_prop->param_size = 31;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 7, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_morphology_sym);
 	_ppvar[5]._pval = &prop_ion->param[0]; /* diam */
 prop_ion = need_memb(_ca_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[2]._pval = &prop_ion->param[1]; /* cai */
 	_ppvar[3]._pval = &prop_ion->param[3]; /* ica */
 	_ppvar[4]._pval = &prop_ion->param[4]; /* _ion_dicadv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _net_receive(Point_process*, double*, double);
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _glutamate1b_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("ca", -10000.);
 	_morphology_sym = hoc_lookup("morphology");
 	_ca_sym = hoc_lookup("ca_ion");
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 1,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 31, 7);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "cvodeieq");
  hoc_register_dparam_semantics(_mechtype, 5, "diam");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 glutamate1b /home/adamimos/Documents/Projects/Godenzini/Archive/glutamate1b.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double F = 96480.0;
 static double R = 8.314;
 static double PI = 3.14159;
static int _reset;
static char *modelname = "NMDA synapse with depression";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[5], _dlist1[5];
 static int state(_threadargsproto_);
 
static void _net_receive (_pnt, _args, _lflag) Point_process* _pnt; double* _args; double _lflag; 
{  double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _thread = (Datum*)0; _nt = (_NrnThread*)_pnt->_vnt;   _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t; {
       if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = A;
    double __primary = (A + gnmdamax * ( dnmda ) ) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau1 ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau1 ) - __primary );
    A += __primary;
  } else {
 A = A + gnmdamax * ( dnmda )  ;
     }
     if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = B;
    double __primary = (B + gnmdamax * ( dnmda ) ) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau2 ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau2 ) - __primary );
    B += __primary;
  } else {
 B = B + gnmdamax * ( dnmda )  ;
     }
     if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = gampa;
    double __primary = (gampa + gampamax * dampa ) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau_ampa ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau_ampa ) - __primary );
    gampa += __primary;
  } else {
 gampa = gampa + gampamax * dampa  ;
     }
     if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = dampa;
    double __primary = (dampa * decayampa ) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( ( ( - 1.0 ) ) ) / taudampa ) ) )*( - ( ( ( 1.0 ) ) / taudampa ) / ( ( ( ( - 1.0 ) ) ) / taudampa ) - __primary );
    dampa += __primary;
  } else {
 dampa = dampa * decayampa  ;
     }
     if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = dnmda;
    double __primary = (dnmda * decaynmda ) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( ( ( - 1.0 ) ) ) / taudnmda ) ) )*( - ( ( ( 1.0 ) ) / taudnmda ) / ( ( ( ( - 1.0 ) ) ) / taudnmda ) - __primary );
    dnmda += __primary;
  } else {
 dnmda = dnmda * decaynmda  ;
     }
 } }
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   DA = - A / tau1 ;
   DB = - B / tau2 ;
   Dgampa = - gampa / tau_ampa ;
   Ddampa = ( 1.0 - dampa ) / taudampa ;
   Ddnmda = ( 1.0 - dnmda ) / taudnmda ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 DA = DA  / (1. - dt*( ( - 1.0 ) / tau1 )) ;
 DB = DB  / (1. - dt*( ( - 1.0 ) / tau2 )) ;
 Dgampa = Dgampa  / (1. - dt*( ( - 1.0 ) / tau_ampa )) ;
 Ddampa = Ddampa  / (1. - dt*( ( ( ( - 1.0 ) ) ) / taudampa )) ;
 Ddnmda = Ddnmda  / (1. - dt*( ( ( ( - 1.0 ) ) ) / taudnmda )) ;
  return 0;
}
 /*END CVODE*/
 static int state (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
    A = A + (1. - exp(dt*(( - 1.0 ) / tau1)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau1 ) - A) ;
    B = B + (1. - exp(dt*(( - 1.0 ) / tau2)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau2 ) - B) ;
    gampa = gampa + (1. - exp(dt*(( - 1.0 ) / tau_ampa)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau_ampa ) - gampa) ;
    dampa = dampa + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / taudampa)))*(- ( ( ( 1.0 ) ) / taudampa ) / ( ( ( ( - 1.0 ) ) ) / taudampa ) - dampa) ;
    dnmda = dnmda + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / taudnmda)))*(- ( ( ( 1.0 ) ) / taudnmda ) / ( ( ( ( - 1.0 ) ) ) / taudnmda ) - dnmda) ;
   }
  return 0;
}
 
static int _ode_count(int _type){ return 5;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  cai = _ion_cai;
     _ode_spec1 (_p, _ppvar, _thread, _nt);
  }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 5; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  cai = _ion_cai;
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca_sym, _ppvar, 2, 1);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 3, 3);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 4, 4);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  A = A0;
  B = B0;
  dnmda = dnmda0;
  dampa = dampa0;
  gampa = gampa0;
 {
   gnmda = 0.0 ;
   gampa = 0.0 ;
   A = 0.0 ;
   B = 0.0 ;
   dampa = 1.0 ;
   dnmda = 1.0 ;
   ica = 0.0 ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _tsav = -1e20;
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
  cai = _ion_cai;
 initmodel(_p, _ppvar, _thread, _nt);
 }
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   double _lcount ;
 local_v = v * ( 1.0 - Voff ) + Vset * Voff ;
   gnmda = ( A - B ) / ( 1.0 + n * exp ( - gama * local_v ) ) ;
   inmda = ( 1e-3 ) * gnmda * ( v - e ) ;
   iampa = ( 1e-3 ) * gampa * ( v - e ) ;
   local_v = v ;
   local_ca = cai ;
   ica = inmda * 0.1 / ( PI * diam ) * icaconst ;
   inmda = inmda * .9 ;
   }
 _current += inmda;
 _current += iampa;
 _current += ica;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
  cai = _ion_cai;
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ double _dica;
  _dica = ica;
 _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
  _ion_dicadv += (_dica - ica)/.001 * 1.e2/ (_nd_area);
 	}
 _g = (_g - _rhs)/.001;
  _ion_ica += ica * 1.e2/ (_nd_area);
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
  cai = _ion_cai;
 {   state(_p, _ppvar, _thread, _nt);
  } }}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(A) - _p;  _dlist1[0] = &(DA) - _p;
 _slist1[1] = &(B) - _p;  _dlist1[1] = &(DB) - _p;
 _slist1[2] = &(gampa) - _p;  _dlist1[2] = &(Dgampa) - _p;
 _slist1[3] = &(dampa) - _p;  _dlist1[3] = &(Ddampa) - _p;
 _slist1[4] = &(dnmda) - _p;  _dlist1[4] = &(Ddnmda) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "/home/adamimos/Documents/Projects/Godenzini/Archive/glutamate1b.mod";
static const char* nmodl_file_text = 
  "COMMENT\n"
  "//****************************//\n"
  "// Created by Alon Polsky 	//\n"
  "//    apmega@yahoo.com		//\n"
  "//		2010			//\n"
  "//****************************//\n"
  "ENDCOMMENT\n"
  "\n"
  "TITLE NMDA synapse with depression\n"
  "\n"
  "\n"
  "NEURON {\n"
  "	POINT_PROCESS glutamate1b\n"
  "	NONSPECIFIC_CURRENT inmda,iampa\n"
  "	RANGE Voff,Vset\n"
  "	RANGE e ,gampamax,gnmdamax,local_v,inmda,iampa,local_ca\n"
  "	RANGE decayampa,decaynmda,dampa,dnmda\n"
  "	RANGE gnmda,gampa,xloc,yloc,tag1,tag2\n"
  "	GLOBAL n, gama,tau_ampa,taudampa,taudnmda,tau1,tau2\n"
  "	GLOBAL icaconst\n"
  "	USEION ca READ cai WRITE ica\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "	(nA) 	= (nanoamp)\n"
  "	(mV)	= (millivolt)\n"
  "	(nS) 	= (nanomho)\n"
  "	(mM)    = (milli/liter)\n"
  "        F	= 96480 (coul)\n"
  "        R       = 8.314 (volt-coul/degC)\n"
  " 	PI = (pi) (1)\n"
  " 	(mA) = (milliamp)\n"
  "	(um) = (micron)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	gnmdamax=1	(nS)\n"
  "	gampamax=1	(nS)\n"
  "	icaconst =0.1:1e-6\n"
  "	e= 0.0	(mV)\n"
  "	tau1=50	(ms)	\n"
  "	tau2=2	(ms)	\n"
  "	tau_ampa=2	(ms)	\n"
  "	n=0.25 	(/mM)	\n"
  "	gama=0.08 	(/mV) \n"
  "	dt 		(ms)\n"
  "	v		(mV)\n"
  ":	del=30	(ms)\n"
  ":	Tspike=10	(ms)\n"
  ":	Nspike=1\n"
  "	Voff=0		:0 - voltage dependent 1- voltage independent\n"
  "	Vset=-60		:set voltage when voltage independent\n"
  "	decayampa=.5\n"
  "	decaynmda=.5\n"
  "	taudampa=200	(ms):tau decay\n"
  "	taudnmda=200	(ms):tau decay\n"
  "\n"
  "	xloc=0\n"
  "	yloc=0\n"
  "	tag1=0\n"
  "	tag2=0\n"
  "}\n"
  "\n"
  "ASSIGNED { \n"
  "	inmda		(nA)  \n"
  "	iampa		(nA)  \n"
  "	gnmda		(nS)\n"
  "	local_v	(mV):local voltage\n"
  "	local_ca	:local calcium,\n"
  "	ica			(nA)\n"
  "	cai\n"
  "\n"
  "}\n"
  "STATE {\n"
  "	A 		(nS)\n"
  "	B 		(nS)\n"
  "	gampa 	(nS)\n"
  "	dampa\n"
  "	dnmda\n"
  "}\n"
  "\n"
  "\n"
  "INITIAL {\n"
  "      gnmda=0 \n"
  "      gampa=0 \n"
  "	A=0\n"
  "	B=0\n"
  "	dampa=1\n"
  "	dnmda=1\n"
  "	ica=0\n"
  "}    \n"
  "\n"
  "BREAKPOINT {  \n"
  "    \n"
  "	LOCAL count\n"
  "	SOLVE state METHOD cnexp\n"
  "	local_v=v*(1-Voff)+Vset*Voff\n"
  "	gnmda=(A-B)/(1+n*exp(-gama*local_v) )\n"
  "	inmda =(1e-3)*gnmda*(v-e)\n"
  "	iampa= (1e-3)*gampa*(v- e)\n"
  "	local_v=v\n"
  "	local_ca=cai\n"
  "	ica=inmda*0.1/(PI*diam)*icaconst\n"
  "	inmda=inmda*.9\n"
  "\n"
  "}\n"
  "NET_RECEIVE(weight) {\n"
  "	state_discontinuity( A, A+ gnmdamax*(dnmda))\n"
  "	state_discontinuity( B, B+ gnmdamax*(dnmda))\n"
  "	state_discontinuity( gampa, gampa+ gampamax*dampa)\n"
  "	state_discontinuity( dampa, dampa* decayampa)\n"
  "	state_discontinuity( dnmda, dnmda* decaynmda)\n"
  "}\n"
  "DERIVATIVE state {\n"
  "	A'=-A/tau1\n"
  "	B'=-B/tau2\n"
  "	gampa'=-gampa/tau_ampa\n"
  "	dampa'=(1-dampa)/taudampa\n"
  "	dnmda'=(1-dnmda)/taudnmda\n"
  "}\n"
  ;
#endif
