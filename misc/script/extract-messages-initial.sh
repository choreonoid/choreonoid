#!/bin/sh

if [ ! -d "po" ]; then
    mkdir po
fi

echo "Extracting messages to translate from the source files."
xgettext -k"_" -k"N_" -k"Q_" -o po/messages.pot *.cpp

if [ -n "$1" ]; then
    echo "Creating $1.po file."
    msginit --input=po/messages.pot --locale=$1 -o po/$1.po
else
    echo "Please specify a locale."
fi
