/****************************************************************************
** Meta object code from reading C++ file 'parameterDialog.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "parameterDialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'parameterDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ParameterDialog[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      17,   16,   16,   16, 0x0a,
      24,   16,   16,   16, 0x0a,
      45,   16,   16,   16, 0x0a,
      71,   16,   16,   16, 0x08,
      96,   16,   16,   16, 0x08,
     123,   16,   16,   16, 0x08,
     147,   16,   16,   16, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_ParameterDialog[] = {
    "ParameterDialog\0\0init()\0updateField(QString)\0"
    "modifyParameters(QString)\0"
    "on_resetButton_clicked()\0"
    "on_guardarButton_clicked()\0"
    "on_buttonBox_accepted()\0"
    "on_createButton_clicked()\0"
};

void ParameterDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ParameterDialog *_t = static_cast<ParameterDialog *>(_o);
        switch (_id) {
        case 0: _t->init(); break;
        case 1: _t->updateField((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->modifyParameters((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->on_resetButton_clicked(); break;
        case 4: _t->on_guardarButton_clicked(); break;
        case 5: _t->on_buttonBox_accepted(); break;
        case 6: _t->on_createButton_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData ParameterDialog::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ParameterDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_ParameterDialog,
      qt_meta_data_ParameterDialog, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ParameterDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ParameterDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ParameterDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ParameterDialog))
        return static_cast<void*>(const_cast< ParameterDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int ParameterDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
