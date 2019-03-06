#!/bin/sh
echo "Extracting messages to translate from the source files."
xgettext -k"_" -k"N_" -k"Q_" -o po/messages.pot `find . -name '*.cpp'`

for po in po/*.po
do
    echo "Merging new messages to $po."
    msgmerge --update $po po/messages.pot
done
