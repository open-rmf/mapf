#!/bin/bash
set -o errexit

read -p "Do you really want to deploy to GitHub Pages? (y/N) "
echo  # send a carriage return
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Exiting..."
    exit 1
fi

read -p "Which branch? [main] ? "
echo # send a carriage return
BRANCH=$REPLY
if [[ -z $BRANCH ]]; then
    BRANCH="main"
fi

echo "Starting deployment process using branch $BRANCH"

set -o verbose

git clone ssh://git@github.com/open-rmf/mapf temp-deploy-checkout --branch $BRANCH --single-branch --depth 1

cd temp-deploy-checkout
git checkout --orphan gh-pages
git reset
scripts/build-web.sh

git add -f web
cp web/root_index.html index.html
git add index.html

git commit -a -m "publish to github pages"

git push origin gh-pages --force
