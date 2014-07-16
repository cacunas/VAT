/****************************************************************************
** Meta object code from reading C++ file 'qwt_polar_picker.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "QWT/qwt_polar_picker.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qwt_polar_picker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_QwtPolarPicker[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: signature, parameters, type, tag, flags
      20,   16,   15,   15, 0x05,
      51,   44,   15,   15, 0x05,
      84,   16,   15,   15, 0x05,
     108,   16,   15,   15, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_QwtPolarPicker[] = {
    "QwtPolarPicker\0\0pos\0selected(QwtPointPolar)\0"
    "points\0selected(QVector<QwtPointPolar>)\0"
    "appended(QwtPointPolar)\0moved(QwtPointPolar)\0"
};

void QwtPolarPicker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QwtPolarPicker *_t = static_cast<QwtPolarPicker *>(_o);
        switch (_id) {
        case 0: _t->selected((*reinterpret_cast< const QwtPointPolar(*)>(_a[1]))); break;
        case 1: _t->selected((*reinterpret_cast< const QVector<QwtPointPolar>(*)>(_a[1]))); break;
        case 2: _t->appended((*reinterpret_cast< const QwtPointPolar(*)>(_a[1]))); break;
        case 3: _t->moved((*reinterpret_cast< const QwtPointPolar(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData QwtPolarPicker::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject QwtPolarPicker::staticMetaObject = {
    { &QwtPicker::staticMetaObject, qt_meta_stringdata_QwtPolarPicker,
      qt_meta_data_QwtPolarPicker, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &QwtPolarPicker::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *QwtPolarPicker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *QwtPolarPicker::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_QwtPolarPicker))
        return static_cast<void*>(const_cast< QwtPolarPicker*>(this));
    return QwtPicker::qt_metacast(_clname);
}

int QwtPolarPicker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QwtPicker::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void QwtPolarPicker::selected(const QwtPointPolar & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void QwtPolarPicker::selected(const QVector<QwtPointPolar> & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void QwtPolarPicker::appended(const QwtPointPolar & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void QwtPolarPicker::moved(const QwtPointPolar & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_END_MOC_NAMESPACE
